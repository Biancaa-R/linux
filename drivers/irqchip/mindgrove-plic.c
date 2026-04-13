// SPDX-License-Identifier: GPL-2.0
/*
 * Mindgrove Platform-Level Interrupt Controller (PLIC) Driver
 * 
 * Author : Biancaa Ramesh <biancaa2210329@ssn.edu.in>
 *
 * Copyright (C) 2026 Mindgrove Technologies
 *
 * Hardware register layout (offsets from PLIC base):
 *
 *   0x000000  PRIORITY[82]      - u32 per source, priority 0-7
 *   0x001000  PENDING_0_31      - bitmap, sources 0-31
 *   0x001004  PENDING_32_63     - bitmap, sources 32-63
 *   0x001008  PENDING_64_81     - bitmap, sources 64-81 (18 bits valid)
 *   0x002000  INTR_EN_0_31      - enable bitmap, sources 0-31
 *   0x002004  INTR_EN_32_63     - enable bitmap, sources 32-63
 *   0x002008  INTR_EN_64_81     - enable bitmap, sources 64-81 (18 bits valid)
 *   0x200000  PRIORITY_THRESH   - u32, CPU ignores IRQs <= this priority
 *   0x200004  INTR_COMPLETE     - read=claim IRQ id, write=complete IRQ
 *
 * Single hart, single context — no per-hart stride.
 * Claim/complete is a single register: read it to get the pending IRQ,
 * write the IRQ id back to signal completion.
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <asm/smp.h>

/* -----------------------------------------------------------------------
 * Register offsets
 * --------------------------------------------------------------------- */

#define MG_PLIC_NUM_SOURCES         82

#define MG_PLIC_PRIORITY_BASE       0x000000
#define MG_PLIC_PRIORITY_PER_ID     4

#define MG_PLIC_PENDING_0_31        0x001000
#define MG_PLIC_PENDING_32_63       0x001004
#define MG_PLIC_PENDING_64_81       0x001008

#define MG_PLIC_INTR_EN_0_31        0x002000
#define MG_PLIC_INTR_EN_32_63       0x002004
#define MG_PLIC_INTR_EN_64_81       0x002008

#define MG_PLIC_THRESHOLD           0x200000
#define MG_PLIC_CLAIM_COMPLETE      0x200004

#define MG_PLIC_DISABLE_THRESHOLD   0x7
#define MG_PLIC_ENABLE_THRESHOLD    0x0
#define MG_PLIC_DEFAULT_PRIORITY    0x1

/* -----------------------------------------------------------------------
 * Driver private data
 * --------------------------------------------------------------------- */

struct mg_plic_priv {
	void __iomem		*regs;
	struct irq_domain	*irqdomain;
	raw_spinlock_t		enable_lock;
	/* Shadow copies of enable registers for atomic RMW */
	u32			enable_save[3];  /* [0]=0-31 [1]=32-63 [2]=64-81 */
};

/* -----------------------------------------------------------------------
 * Low-level register helpers
 * --------------------------------------------------------------------- */

/*
 * Return pointer to the enable register and the bit mask for a given hwirq.
 * Also returns the index into enable_save[].
 */
static void __iomem *mg_plic_enable_reg(struct mg_plic_priv *priv,
					int hwirq, u32 *mask, int *idx)
{
	if (hwirq < 32) {
		*mask = BIT(hwirq);
		*idx  = 0;
		return priv->regs + MG_PLIC_INTR_EN_0_31;
	} else if (hwirq < 64) {
		*mask = BIT(hwirq - 32);
		*idx  = 1;
		return priv->regs + MG_PLIC_INTR_EN_32_63;
	} else {
		*mask = BIT(hwirq - 64);
		*idx  = 2;
		return priv->regs + MG_PLIC_INTR_EN_64_81;
	}
}

/* Enable or disable a single interrupt source. Must hold enable_lock. */
static void __mg_plic_toggle(struct mg_plic_priv *priv, int hwirq, int enable)
{
	void __iomem *reg;
	u32 mask;
	int idx;

	reg = mg_plic_enable_reg(priv, hwirq, &mask, &idx);

	if (enable)
		priv->enable_save[idx] |= mask;
	else
		priv->enable_save[idx] &= ~mask;

	writel(priv->enable_save[idx], reg);
}

static void mg_plic_toggle(struct mg_plic_priv *priv, int hwirq, int enable)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&priv->enable_lock, flags);
	__mg_plic_toggle(priv, hwirq, enable);
	raw_spin_unlock_irqrestore(&priv->enable_lock, flags);
}

static void mg_plic_set_priority(struct mg_plic_priv *priv, int hwirq, u32 prio)
{
	writel(prio, priv->regs + MG_PLIC_PRIORITY_BASE +
	       hwirq * MG_PLIC_PRIORITY_PER_ID);
}

static u32 mg_plic_claim(struct mg_plic_priv *priv)
{
	return readl(priv->regs + MG_PLIC_CLAIM_COMPLETE);
}

static void mg_plic_complete(struct mg_plic_priv *priv, u32 hwirq)
{
	writel(hwirq, priv->regs + MG_PLIC_CLAIM_COMPLETE);
}

/* -----------------------------------------------------------------------
 * irq_chip callbacks
 * --------------------------------------------------------------------- */

static void mg_plic_irq_mask(struct irq_data *d)
{
	struct mg_plic_priv *priv = irq_data_get_irq_chip_data(d);

	mg_plic_toggle(priv, d->hwirq, 0);
}

static void mg_plic_irq_unmask(struct irq_data *d)
{
	struct mg_plic_priv *priv = irq_data_get_irq_chip_data(d);

	mg_plic_toggle(priv, d->hwirq, 1);
}

/*
 * irq_enable: unmask + set priority to 1 so interrupt can fire.
 * irq_disable: mask + set priority to 0 as belt-and-suspenders.
 */
static void mg_plic_irq_enable(struct irq_data *d)
{
	struct mg_plic_priv *priv = irq_data_get_irq_chip_data(d);

	mg_plic_set_priority(priv, d->hwirq, MG_PLIC_DEFAULT_PRIORITY);
	mg_plic_toggle(priv, d->hwirq, 1);
}

static void mg_plic_irq_disable(struct irq_data *d)
{
	struct mg_plic_priv *priv = irq_data_get_irq_chip_data(d);

	mg_plic_toggle(priv, d->hwirq, 0);
	mg_plic_set_priority(priv, d->hwirq, 0);
}

/*
 * EOI: write the hwirq back to INTR_COMPLETE to signal we are done.
 * This re-arms the interrupt for the next delivery.
 */
static void mg_plic_irq_eoi(struct irq_data *d)
{
	struct mg_plic_priv *priv = irq_data_get_irq_chip_data(d);

	mg_plic_complete(priv, d->hwirq);
}

static struct irq_chip mg_plic_chip = {
	.name		= "mindgrove-plic",
	.irq_enable	= mg_plic_irq_enable,
	.irq_disable	= mg_plic_irq_disable,
	.irq_mask	= mg_plic_irq_mask,
	.irq_unmask	= mg_plic_irq_unmask,
	.irq_eoi	= mg_plic_irq_eoi,
	.flags		= IRQCHIP_SKIP_SET_WAKE | IRQCHIP_AFFINITY_PRE_STARTUP,
};

/* -----------------------------------------------------------------------
 * Chained interrupt handler (called from parent INTC)
 * --------------------------------------------------------------------- */

static void mg_plic_handle_irq(struct irq_desc *desc)
{
	struct mg_plic_priv *priv = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 hwirq;

	chained_irq_enter(chip, desc);

	/*
	 * Read INTR_COMPLETE to claim the highest-priority pending interrupt.
	 * Returns 0 when no more interrupts are pending.
	 * Write back happens in mg_plic_irq_eoi after the handler runs.
	 */
	while ((hwirq = mg_plic_claim(priv))) {
		if (unlikely(hwirq >= MG_PLIC_NUM_SOURCES)) {
			pr_warn_ratelimited("mindgrove-plic: spurious hwirq %u\n",
					    hwirq);
			mg_plic_complete(priv, hwirq);
			break;
		}

		if (generic_handle_domain_irq(priv->irqdomain, hwirq)) {
			pr_warn_ratelimited(
				"mindgrove-plic: no handler for hwirq %u\n",
				hwirq);
			mg_plic_complete(priv, hwirq);
		}
	}

	chained_irq_exit(chip, desc);
}

/* -----------------------------------------------------------------------
 * IRQ domain ops
 * --------------------------------------------------------------------- */

static int mg_plic_domain_map(struct irq_domain *d, unsigned int irq,
			      irq_hw_number_t hwirq)
{
	struct mg_plic_priv *priv = d->host_data;

	irq_domain_set_info(d, irq, hwirq, &mg_plic_chip, priv,
			    handle_fasteoi_irq, NULL, NULL);
	irq_set_noprobe(irq);
	return 0;
}

static int mg_plic_domain_translate(struct irq_domain *d,
				    struct irq_fwspec *fwspec,
				    unsigned long *hwirq,
				    unsigned int *type)
{
	if (WARN_ON(fwspec->param_count < 1))
		return -EINVAL;

	*hwirq = fwspec->param[0];
	*type  = IRQ_TYPE_LEVEL_HIGH;
	return 0;
}

static int mg_plic_domain_alloc(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs, void *arg)
{
	struct irq_fwspec *fwspec = arg;
	irq_hw_number_t hwirq;
	unsigned int type;
	int i, ret;

	ret = mg_plic_domain_translate(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	for (i = 0; i < nr_irqs; i++) {
		ret = mg_plic_domain_map(domain, virq + i, hwirq + i);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct irq_domain_ops mg_plic_domain_ops = {
	.translate	= mg_plic_domain_translate,
	.alloc		= mg_plic_domain_alloc,
	.free		= irq_domain_free_irqs_top,
};

/* -----------------------------------------------------------------------
 * Hardware initialisation
 * --------------------------------------------------------------------- */

static void mg_plic_hw_init(struct mg_plic_priv *priv)
{
	int i;

	/* Disable all interrupt sources at the enable registers */
	writel(0, priv->regs + MG_PLIC_INTR_EN_0_31);
	writel(0, priv->regs + MG_PLIC_INTR_EN_32_63);
	writel(0, priv->regs + MG_PLIC_INTR_EN_64_81);

	/* Clear shadow copies */
	priv->enable_save[0] = 0;
	priv->enable_save[1] = 0;
	priv->enable_save[2] = 0;

	/* Set all priorities to 0 (disabled) */
	for (i = 0; i < MG_PLIC_NUM_SOURCES; i++)
		mg_plic_set_priority(priv, i, 0);

	/* Accept all interrupts above priority 0 */
	writel(MG_PLIC_ENABLE_THRESHOLD, priv->regs + MG_PLIC_THRESHOLD);

	/* Drain any stale claims */
	{
		u32 hwirq;
		while ((hwirq = mg_plic_claim(priv)))
			mg_plic_complete(priv, hwirq);
	}
}

/* -----------------------------------------------------------------------
 * Probe
 * --------------------------------------------------------------------- */

static int mg_plic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct mg_plic_priv *priv;
	struct resource *res;
	unsigned long hartid;
	int parent_irq, cpu, ret;
	struct irq_domain *parent_domain;
	struct of_phandle_args parent;

	pr_info("mindgrove-plic: probing\n");

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	raw_spin_lock_init(&priv->enable_lock);

	/* Map PLIC registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->regs)) {
		dev_err(dev, "failed to map registers\n");
		return PTR_ERR(priv->regs);
	}

	/* Initialise hardware to a clean state */
	mg_plic_hw_init(priv);

	/* Create linear IRQ domain for MG_PLIC_NUM_SOURCES sources */
	priv->irqdomain = irq_domain_add_linear(node,
						MG_PLIC_NUM_SOURCES,
						&mg_plic_domain_ops,
						priv);
	if (!priv->irqdomain) {
		dev_err(dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}

	/*
	 * Find the parent INTC (riscv,cpu-intc) and connect the chained
	 * handler to the external interrupt line (RV_IRQ_EXT = 11).
	 *
	 * The DTS must have:
	 *   interrupts-extended = <&CPU0_intc 11 &CPU0_intc 9>;
	 * We use context 0 (M-mode ext irq) or context 1 (S-mode ext irq)
	 * depending on which privilege level we are running at.
	 * For Linux in S-mode, context 1 (S-mode, hwirq 9) is correct.
	 */
	ret = of_irq_parse_one(node, 1, &parent); /* index 1 = S-mode */
	if (ret) {
		/* Fall back to context 0 if only one context declared */
		ret = of_irq_parse_one(node, 0, &parent);
		if (ret) {
			dev_err(dev, "failed to parse parent IRQ\n");
			goto fail_domain;
		}
	}

	ret = riscv_of_parent_hartid(parent.np, &hartid);
	if (ret) {
		dev_err(dev, "failed to get parent hart id\n");
		goto fail_domain;
	}

	cpu = riscv_hartid_to_cpuid(hartid);
	if (cpu < 0) {
		dev_err(dev, "invalid cpu for hartid %lu\n", hartid);
		ret = -EINVAL;
		goto fail_domain;
	}

	parent_domain = irq_find_matching_fwnode(
		&parent.np->fwnode, DOMAIN_BUS_ANY);
	if (!parent_domain) {
		dev_err(dev, "failed to find parent IRQ domain\n");
		ret = -ENODEV;
		goto fail_domain;
	}

	parent_irq = irq_create_mapping(parent_domain, parent.args[0]);
	if (!parent_irq) {
		dev_err(dev, "failed to map parent IRQ\n");
		ret = -ENODEV;
		goto fail_domain;
	}

	irq_set_chained_handler_and_data(parent_irq, mg_plic_handle_irq, priv);

	platform_set_drvdata(pdev, priv);

	dev_info(dev, "mapped %d interrupt sources, parent irq %d\n",
		 MG_PLIC_NUM_SOURCES, parent_irq);
	return 0;

fail_domain:
	irq_domain_remove(priv->irqdomain);
	return ret;
}

static void mg_plic_remove(struct platform_device *pdev)
{
	struct mg_plic_priv *priv = platform_get_drvdata(pdev);

	irq_domain_remove(priv->irqdomain);
}

/* -----------------------------------------------------------------------
 * Early (IRQCHIP_DECLARE) probe — called before platform bus is up
 * --------------------------------------------------------------------- */

static int __init mg_plic_early_probe(struct device_node *node,
				      struct device_node *parent)
{
	struct mg_plic_priv *priv;
	struct of_phandle_args pa;
	struct irq_domain *parent_domain;
	unsigned long hartid;
	int parent_irq, cpu, ret;
	void __iomem *regs;

	pr_info("mindgrove-plic: early probe\n");

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	raw_spin_lock_init(&priv->enable_lock);

	regs = of_iomap(node, 0);
	if (!regs) {
		pr_err("mindgrove-plic: failed to map registers\n");
		ret = -ENOMEM;
		goto fail_free;
	}
	priv->regs = regs;

	mg_plic_hw_init(priv);

	priv->irqdomain = irq_domain_add_linear(node,
						MG_PLIC_NUM_SOURCES,
						&mg_plic_domain_ops,
						priv);
	if (!priv->irqdomain) {
		pr_err("mindgrove-plic: failed to create IRQ domain\n");
		ret = -ENOMEM;
		goto fail_unmap;
	}

	/* Try S-mode context first (index 1), fall back to index 0 */
	ret = of_irq_parse_one(node, 1, &pa);
	if (ret)
		ret = of_irq_parse_one(node, 0, &pa);
	if (ret) {
		pr_err("mindgrove-plic: no parent IRQ\n");
		goto fail_domain;
	}

	ret = riscv_of_parent_hartid(pa.np, &hartid);
	if (ret) {
		pr_err("mindgrove-plic: failed to get hartid\n");
		goto fail_domain;
	}

	cpu = riscv_hartid_to_cpuid(hartid);
	if (cpu < 0) {
		pr_err("mindgrove-plic: invalid cpu\n");
		ret = -EINVAL;
		goto fail_domain;
	}

	parent_domain = irq_find_matching_fwnode(
		&pa.np->fwnode, DOMAIN_BUS_ANY);
	if (!parent_domain) {
		pr_err("mindgrove-plic: no parent domain\n");
		ret = -ENODEV;
		goto fail_domain;
	}

	parent_irq = irq_create_mapping(parent_domain, pa.args[0]);
	if (!parent_irq) {
		pr_err("mindgrove-plic: failed to map parent IRQ\n");
		ret = -ENODEV;
		goto fail_domain;
	}

	irq_set_chained_handler_and_data(parent_irq, mg_plic_handle_irq, priv);

	pr_info("mindgrove-plic: mapped %d sources, parent irq %d\n",
		MG_PLIC_NUM_SOURCES, parent_irq);
	return 0;

fail_domain:
	irq_domain_remove(priv->irqdomain);
fail_unmap:
	iounmap(regs);
fail_free:
	kfree(priv);
	return ret;
}

/* -----------------------------------------------------------------------
 * Platform driver registration
 * --------------------------------------------------------------------- */

static const struct of_device_id mg_plic_match[] = {
	{ .compatible = "mindgrove,plic" },
	{ }
};
MODULE_DEVICE_TABLE(of, mg_plic_match);

static struct platform_driver mg_plic_driver = {
	.driver = {
		.name			= "mindgrove-plic",
		.of_match_table		= mg_plic_match,
		.suppress_bind_attrs	= true,
	},
	.probe		= mg_plic_probe,
	.remove	= mg_plic_remove,
};
builtin_platform_driver(mg_plic_driver);

/*
 * IRQCHIP_DECLARE registers the early probe so the PLIC is available
 * before the platform bus initialises — required for console IRQs.
 */
IRQCHIP_DECLARE(mindgrove_plic, "mindgrove,plic", mg_plic_early_probe);