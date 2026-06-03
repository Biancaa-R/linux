// SPDX-License-Identifier: GPL-2.0
/*
 * Mindgrove SPI controller Linux driver
 *
 * Copyright (C) 2025 Mindgrove Technologies Private Limited
 *
 * KEY HARDWARE CONSTRAINT (discovered via oscilloscope):
 *
 *   NCS_OUTEN (CTRL bit 23) must be SET for the SW bit in NCS_CTRL to
 *   physically drive the NCS/CS pin.  When CTRL = 0 (engine disabled),
 *   NCS_OUTEN = 0 and the pin is not driven at all — writing SW in
 *   NCS_CTRL has no effect on the wire.
 *
 * Fix: after init, keep CTRL = NCS_OUTEN | SCLK_OUTEN | MOSI_OUTEN at
 * all times (output-enables always on).  When disabling the engine after
 * a transfer, only clear EN (bit 1).  Never write 0x0 to CTRL again after
 * the initial RX-flush sequence.
 *
 * CS is owned exclusively by set_cs() (called by the SPI framework).
 * transfer_one does NOT touch NCS_CTRL.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/vmalloc.h>

#define MINDGROVE_SPI_DRIVER_NAME	"mindgrove_spi"

#define MINDGROVE_SPI_MAX_CS		4
#define MINDGROVE_SPI_DEFAULT_DEPTH	32
#define MINDGROVE_SPI_DEFAULT_BITS	8
#define MINDGROVE_SPI_TIMEOUT_US	1000000
#define MINDGROVE_SPI_MAX_FREQ		30000000
#define MINDGROVE_SPI_INIT_PRESCALER	4u
#define MINDGROVE_SPI_YIELD_INTERVAL	64

/* Register offsets */
#define MINDGROVE_SPI_REG_CTRL		0x00
#define MINDGROVE_SPI_REG_CLK_CTRL	0x04
#define MINDGROVE_SPI_REG_TX		0x08
#define MINDGROVE_SPI_REG_RX		0x0C
#define MINDGROVE_SPI_REG_INTR_EN	0x10
#define MINDGROVE_SPI_REG_FIFO_STATUS	0x14
#define MINDGROVE_SPI_REG_COMM_STATUS	0x18	/* 16-bit */
#define MINDGROVE_SPI_REG_NCS_CTRL	0x1C	/* 32-bit */

/* CTRL register bits */
#define MINDGROVE_SPI_CTRL_SLAVE_MODE(x)	((x) << 0)
#define MINDGROVE_SPI_CTRL_EN(x)		((x) << 1)
#define MINDGROVE_SPI_CTRL_LSBFIRST(x)		((x) << 2)
#define MINDGROVE_SPI_CTRL_RX_FLUSH(x)		((x) << 3)
#define MINDGROVE_SPI_CTRL_COMM_MODE(x)	((x) << 4)
#define MINDGROVE_SPI_CTRL_TOTAL_BIT_TX(x)	((x) << 6)
#define MINDGROVE_SPI_CTRL_TOTAL_BIT_RX(x)	((x) << 14)
#define MINDGROVE_SPI_CTRL_SCLK_OUTEN		((u32)BIT(22))
#define MINDGROVE_SPI_CTRL_NCS_OUTEN		((u32)BIT(23))
/* bit 24 = MISO_OUTEN — do NOT set in master mode */
#define MINDGROVE_SPI_CTRL_MOSI_OUTEN		((u32)BIT(25))

/*
 * CTRL_PIN_MASK — output-enable bits that must ALWAYS stay set after init
 * so that set_cs() (which writes NCS_CTRL.SW) physically drives the NCS pin.
 * Only the EN bit is toggled per-transfer.
 */
#define MINDGROVE_SPI_CTRL_PIN_MASK	(MINDGROVE_SPI_CTRL_SCLK_OUTEN | \
					 MINDGROVE_SPI_CTRL_NCS_OUTEN   | \
					 MINDGROVE_SPI_CTRL_MOSI_OUTEN)

/* CLK_CTRL register */
#define MINDGROVE_SPI_CLK_CTRL_POLARITY		BIT(0)
#define MINDGROVE_SPI_CLK_CTRL_PHASE		BIT(1)
#define MINDGROVE_SPI_CLK_CTRL_PRESCALAR_SHIFT	2
#define MINDGROVE_SPI_CLK_CTRL_PRESCALAR_MASK	GENMASK(15, 2)
#define MINDGROVE_SPI_CLK_CTRL_SETUP_SHIFT	16
#define MINDGROVE_SPI_CLK_CTRL_HOLD_SHIFT	24

/* FIFO_STATUS (32-bit) */
#define MINDGROVE_SPI_FIFO_STATUS_TX_EMPTY	BIT(0)
#define MINDGROVE_SPI_FIFO_STATUS_TX_FULL	BIT(8)
#define MINDGROVE_SPI_FIFO_STATUS_RX_EMPTY	BIT(9)

/* COMM_STATUS (16-bit — always use readw) */
#define MINDGROVE_SPI_COMM_STATUS_BUSY		BIT(0)

/* NCS_CTRL (32-bit) */
#define MINDGROVE_SPI_NCS_CTRL_SELECT(x)	((u32)((x) << 0))
#define MINDGROVE_SPI_NCS_CTRL_SW(x)		((u32)((x) << 1))

#define MINDGROVE_SPI_COMM_MODE_FULL_DUPLEX	3

struct mindgrove_spi {
	struct device		*dev;
	struct spi_controller	*ctlr;
	void __iomem		*base;
	phys_addr_t		phys;
	u32			fifo_depth;
	u32			bits_per_word;
	u32			input_clk_hz;
	u32			spi_freq;
	bool			cpol;
	bool			cpha;
	bool			lsb_first;
	u16			prescaler;
	u8			num_cs;
	spinlock_t		lock;
};

/* ------------------------------------------------------------------ */
/* Register helpers                                                     */
/* ------------------------------------------------------------------ */

/*
 * mindgrove_spi_ctrl_disable - clear EN bit, keep output enables.
 *
 * NEVER write 0x0 to CTRL after init — doing so clears NCS_OUTEN and
 * the physical CS pin stops being driven (confirmed on oscilloscope).
 */
static void mindgrove_spi_ctrl_disable(struct mindgrove_spi *spi)
{
	u32 ctrl = readl(spi->base + MINDGROVE_SPI_REG_CTRL);

	ctrl &= ~MINDGROVE_SPI_CTRL_EN(1);	/* clear EN only */
	ctrl |= MINDGROVE_SPI_CTRL_PIN_MASK;	/* always keep output enables */
	writel(ctrl, spi->base + MINDGROVE_SPI_REG_CTRL);
}

/*
 * mindgrove_spi_ensure_pin_mask - restore output-enable bits if cleared.
 *
 * The SPI core may reset CTRL between init_hw and the first transfer
 * (e.g. during spi_register_controller idle state setup). Call this
 * at the start of prepare_message and set_cs to guarantee the pins
 * are always driven correctly before any CS or transfer activity.
 */
static void mindgrove_spi_ensure_pin_mask(struct mindgrove_spi *spi)
{
	u32 ctrl = readl(spi->base + MINDGROVE_SPI_REG_CTRL);

	if ((ctrl & MINDGROVE_SPI_CTRL_PIN_MASK) != MINDGROVE_SPI_CTRL_PIN_MASK) {
		ctrl = (ctrl & ~MINDGROVE_SPI_CTRL_EN(1)) | MINDGROVE_SPI_CTRL_PIN_MASK;
		writel(ctrl, spi->base + MINDGROVE_SPI_REG_CTRL);
		dev_warn(spi->dev,
			 "PIN_MASK was incomplete! restored CTRL=0x%08x\n",
			 readl(spi->base + MINDGROVE_SPI_REG_CTRL));
	}
}

/* ------------------------------------------------------------------ */
/* Hardware helpers                                                     */
/* ------------------------------------------------------------------ */

static int mindgrove_spi_wait_complete(struct mindgrove_spi *spi)
{
	u32 timeout = MINDGROVE_SPI_TIMEOUT_US;
	u32 iter = 0;

	while (timeout--) {
		u32 fs = readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS);
		u16 cs = readw(spi->base + MINDGROVE_SPI_REG_COMM_STATUS);

		if ((fs & MINDGROVE_SPI_FIFO_STATUS_TX_EMPTY) &&
		    !(cs & MINDGROVE_SPI_COMM_STATUS_BUSY))
			return 0;

		if (!(++iter % MINDGROVE_SPI_YIELD_INTERVAL))
			cond_resched();
	}
	dev_err(spi->dev, "wait_complete TIMEOUT FIFO=0x%08x COMM=0x%04x\n",
		readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS),
		readw(spi->base + MINDGROVE_SPI_REG_COMM_STATUS));
	return -ETIMEDOUT;
}

/* Caller MUST hold spi->lock */
static int mindgrove_spi_set_speed_locked(struct mindgrove_spi *spi, u32 speed)
{
	u32 prescaler, clk_ctrl, actual_freq;

	if (!speed)
		return 0;

	prescaler = spi->input_clk_hz / speed;
	if (prescaler)
		prescaler -= 1;
	if (prescaler > 0x3FFF)
		prescaler = 0x3FFF;

	actual_freq = spi->input_clk_hz / (prescaler + 1);
	if (actual_freq > MINDGROVE_SPI_MAX_FREQ) {
		dev_err(spi->dev, "freq %u Hz exceeds max %u Hz\n",
			actual_freq, MINDGROVE_SPI_MAX_FREQ);
		return -EINVAL;
	}

	if (prescaler == spi->prescaler)
		return 0;

	spi->prescaler = prescaler;
	spi->spi_freq  = speed;

	clk_ctrl  = readl(spi->base + MINDGROVE_SPI_REG_CLK_CTRL);
	clk_ctrl &= ~MINDGROVE_SPI_CLK_CTRL_PRESCALAR_MASK;
	clk_ctrl |= (prescaler << MINDGROVE_SPI_CLK_CTRL_PRESCALAR_SHIFT);
	writel(clk_ctrl, spi->base + MINDGROVE_SPI_REG_CLK_CTRL);

	dev_dbg(spi->dev,
		 "set_speed: req=%u actual=%u prescaler=%u CLK_CTRL=0x%08x\n",
		 speed, actual_freq, prescaler,
		 readl(spi->base + MINDGROVE_SPI_REG_CLK_CTRL));
	return 0;
}

/*
 * mindgrove_spi_prep — enable the SPI engine for one transfer.
 *
 * Writes all CTRL bits including PIN_MASK (output enables) and EN.
 * This is called from transfer_one after set_cs() has asserted CS.
 */
static void mindgrove_spi_prep(struct mindgrove_spi *spi)
{
	u32 ctrl;

	ctrl  = MINDGROVE_SPI_CTRL_COMM_MODE(MINDGROVE_SPI_COMM_MODE_FULL_DUPLEX);
	ctrl |= MINDGROVE_SPI_CTRL_TOTAL_BIT_TX(spi->bits_per_word);
	ctrl |= MINDGROVE_SPI_CTRL_TOTAL_BIT_RX(spi->bits_per_word);
	ctrl |= MINDGROVE_SPI_CTRL_PIN_MASK;	/* SCLK_OUTEN|NCS_OUTEN|MOSI_OUTEN */
	if (spi->lsb_first)
		ctrl |= MINDGROVE_SPI_CTRL_LSBFIRST(1);
	ctrl |= MINDGROVE_SPI_CTRL_EN(1);

	writel(ctrl, spi->base + MINDGROVE_SPI_REG_CTRL);
	dev_dbg(spi->dev, "prep: CTRL=0x%08x rb=0x%08x\n",
		 ctrl, readl(spi->base + MINDGROVE_SPI_REG_CTRL));
}

static int mindgrove_spi_wait_tx_not_full(struct mindgrove_spi *spi)
{
	u32 timeout = MINDGROVE_SPI_TIMEOUT_US;
	u32 iter = 0;

	while (readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS) &
	       MINDGROVE_SPI_FIFO_STATUS_TX_FULL) {
		if (!timeout--) {
			dev_err(spi->dev, "TX FIFO full TIMEOUT FIFO=0x%08x\n",
				readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS));
			return -ETIMEDOUT;
		}

		if (!(++iter % MINDGROVE_SPI_YIELD_INTERVAL))
			cond_resched();
	}
	return 0;
}

static int mindgrove_spi_wait_rx_not_empty(struct mindgrove_spi *spi)
{
	u32 timeout = MINDGROVE_SPI_TIMEOUT_US;
	u32 iter = 0;

	while (readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS) &
	       MINDGROVE_SPI_FIFO_STATUS_RX_EMPTY) {
		if (!timeout--) {
			dev_err(spi->dev, "RX FIFO empty TIMEOUT FIFO=0x%08x\n",
				readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS));
			return -ETIMEDOUT;
		}

		if (!(++iter % MINDGROVE_SPI_YIELD_INTERVAL))
			cond_resched();
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* SPI controller callbacks                                             */
/* ------------------------------------------------------------------ */

/*
 * mindgrove_spi_set_cs — assert or deassert chip select.
 *
 * Because NCS_OUTEN is always kept set in CTRL (see init_hw and
 * mindgrove_spi_ctrl_disable), writing SW here PHYSICALLY drives the
 * NCS pin immediately, with no need for the engine to be running.
 *
 * NCS_CTRL SW truth table:
 *   SW=0 → NCS pin LOW  → CS asserted   (active-low device, normal case)
 *   SW=1 → NCS pin HIGH → CS deasserted
 *
 *   enable  cs_high  SW    result
 *     T       F       0    assert   (active-low)
 *     T       T       1    assert   (active-high)
 *     F       F       1    deassert (active-low)
 *     F       T       0    deassert (active-high)
 *   → SW = (enable == cs_high)
 */
static void mindgrove_spi_set_cs(struct spi_device *spi_dev, bool enable)
{
	struct mindgrove_spi *spi =
		spi_controller_get_devdata(spi_dev->controller);
	bool cs_high = !!(spi_dev->mode & SPI_CS_HIGH);
	u32 ncs_ctrl;

	/*
	 * Restore PIN_MASK BEFORE writing NCS_CTRL.SW — if NCS_OUTEN is
	 * not set, the SW write has no physical effect on the pin.
	 */
	mindgrove_spi_ensure_pin_mask(spi);

	ncs_ctrl = readl(spi->base + MINDGROVE_SPI_REG_NCS_CTRL);

	/*
	 * SW truth table (verified from U-Boot driver):
	 *   SW=1 → CS asserted   (pin LOW for active-low, pin HIGH for active-high)
	 *   SW=0 → CS deasserted
	 *
	 *   enable  cs_high  SW
	 *     T       F       1   assert   active-low
	 *     T       T       0   assert   active-high  (SW=0 = pin HIGH? No...)
	 *
	 * U-Boot: assert active-low → SW=1, assert active-high → SW=0
	 * So SW = (enable XOR cs_high) ... but simpler: SW = (enable != cs_high)
	 *   enable=T cs_high=F → SW=1  assert  active-low  ✓
	 *   enable=T cs_high=T → SW=0  assert  active-high ✓
	 *   enable=F cs_high=F → SW=0  deassert active-low ✓
	 *   enable=F cs_high=T → SW=1  deassert active-high ✓
	 */
	if (enable != cs_high)
		ncs_ctrl |=  MINDGROVE_SPI_NCS_CTRL_SW(1);	/* SW=1 */
	else
		ncs_ctrl &= ~MINDGROVE_SPI_NCS_CTRL_SW(1);	/* SW=0 */

	writel(ncs_ctrl, spi->base + MINDGROVE_SPI_REG_NCS_CTRL);

	dev_dbg(spi->dev,
		 "set_cs: %s cs=%u cs_high=%d SW=%d "
		 "NCS_CTRL=0x%08x CTRL=0x%08x\n",
		 enable ? "ASSERT  " : "DEASSERT",
		 spi_get_chipselect(spi_dev, 0), cs_high,
		 !!(ncs_ctrl & MINDGROVE_SPI_NCS_CTRL_SW(1)),
		 readl(spi->base + MINDGROVE_SPI_REG_NCS_CTRL),
		 readl(spi->base + MINDGROVE_SPI_REG_CTRL));
}

static int mindgrove_spi_setup(struct spi_device *spi_dev)
{
	struct mindgrove_spi *spi =
		spi_controller_get_devdata(spi_dev->controller);
	unsigned long flags;
	u32 clk_ctrl;

	spin_lock_irqsave(&spi->lock, flags);

	spi->cpha = !!(spi_dev->mode & SPI_CPHA);
	spi->cpol = !!(spi_dev->mode & SPI_CPOL);

	clk_ctrl  = readl(spi->base + MINDGROVE_SPI_REG_CLK_CTRL);
	clk_ctrl &= ~(MINDGROVE_SPI_CLK_CTRL_POLARITY |
		      MINDGROVE_SPI_CLK_CTRL_PHASE);
	if (spi->cpha)
		clk_ctrl |= MINDGROVE_SPI_CLK_CTRL_PHASE;
	if (spi->cpol)
		clk_ctrl |= MINDGROVE_SPI_CLK_CTRL_POLARITY;
	writel(clk_ctrl, spi->base + MINDGROVE_SPI_REG_CLK_CTRL);

	spi->lsb_first = !!(spi_dev->mode & SPI_LSB_FIRST);
	spi->bits_per_word = spi_dev->bits_per_word ?
			     spi_dev->bits_per_word : MINDGROVE_SPI_DEFAULT_BITS;

	dev_dbg(spi->dev,
		 "setup: mode=0x%x cpol=%d cpha=%d lsb=%d bpw=%d "
		 "CLK_CTRL=0x%08x\n",
		 spi_dev->mode, spi->cpol, spi->cpha, spi->lsb_first,
		 spi->bits_per_word,
		 readl(spi->base + MINDGROVE_SPI_REG_CLK_CTRL));

	spin_unlock_irqrestore(&spi->lock, flags);

	/*
	 * Immediately apply CS state to hardware.
	 *
	 * mmc_spi calls spi_setup() with SPI_CS_HIGH XOR'd to toggle CS
	 * polarity for the 74-clock init sequence, then immediately submits
	 * a transfer expecting the CS pin to already reflect the new polarity.
	 *
	 * With software-only CS (SW bit in NCS_CTRL), the pin only changes
	 * when set_cs() is called.  Drive CS to its idle (deasserted) state
	 * here so the pin is correct before any transfer is submitted.
	 */
	mindgrove_spi_set_cs(spi_dev, false);

	return 0;
}

static int mindgrove_spi_prepare_msg(struct spi_controller *ctlr,
				     struct spi_message *msg)
{
	struct mindgrove_spi *spi = spi_controller_get_devdata(ctlr);
	struct spi_device *spi_dev = msg->spi;
	unsigned long flags;
	u32 speed;
	int ret;

	dev_err(spi->dev, "PREPARE_MSG speed=%u CTRL=0x%08x NCS=0x%08x",
		msg->spi->max_speed_hz,
		readl(spi->base + MINDGROVE_SPI_REG_CTRL),
		readl(spi->base + MINDGROVE_SPI_REG_NCS_CTRL));

	spin_lock_irqsave(&spi->lock, flags);

	/* Restore PIN_MASK if cleared — must happen before set_cs is called */
	mindgrove_spi_ensure_pin_mask(spi);

	speed = spi_dev->max_speed_hz ? spi_dev->max_speed_hz : spi->spi_freq;
	if (!speed)
		speed = MINDGROVE_SPI_MAX_FREQ;

	ret = mindgrove_spi_set_speed_locked(spi, speed);
	spin_unlock_irqrestore(&spi->lock, flags);

	return ret;
}

static int mindgrove_spi_unprepare_msg(struct spi_controller *ctlr,
				       struct spi_message *msg)
{
	return 0;
}

/*
 * transfer_one — byte-by-byte PIO transfer.
 *
 * CS is owned by the framework (set_cs before/after).  This function
 * does NOT touch NCS_CTRL.
 *
 * Sequence:
 *   [framework: set_cs(true)  → SW=0 → NCS pin LOW]
 *   1. prep() — enable engine (EN=1, output-enables already set)
 *   2. push/drain bytes
 *   3. wait idle
 *   4. ctrl_disable() — clear EN only, keep output-enables
 *   [framework: set_cs(false) → SW=1 → NCS pin HIGH]
 */
static int mindgrove_spi_transfer_one(struct spi_controller *ctlr,
				      struct spi_device *spi_dev,
				      struct spi_transfer *t)
{
	struct mindgrove_spi *spi = spi_controller_get_devdata(ctlr);
	const u8 *tx_buf = t->tx_buf;
	u8 *rx_buf = t->rx_buf;
	u32 len = t->len;
	unsigned long flags;
	u32 i, speed;
	int ret;

	/* Per-transfer speed override */
	speed = t->speed_hz ? t->speed_hz : spi->spi_freq;
	if (!speed)
		speed = MINDGROVE_SPI_MAX_FREQ;

	spin_lock_irqsave(&spi->lock, flags);
	ret = mindgrove_spi_set_speed_locked(spi, speed);
	spin_unlock_irqrestore(&spi->lock, flags);
	if (ret)
		return ret;

	dev_err(spi->dev, "XFER_ONE len=%u speed=%u CTRL=0x%08x NCS=0x%08x",
		len, speed,
		readl(spi->base + MINDGROVE_SPI_REG_CTRL),
		readl(spi->base + MINDGROVE_SPI_REG_NCS_CTRL));

	dev_dbg(spi->dev,
		 "xfer START: len=%u speed=%u CTRL=0x%08x NCS_CTRL=0x%08x\n",
		 len, speed,
		 readl(spi->base + MINDGROVE_SPI_REG_CTRL),
		 readl(spi->base + MINDGROVE_SPI_REG_NCS_CTRL));

	/* Step 1: enable engine (CS already asserted by framework) */
	mindgrove_spi_prep(spi);

	/* Step 2: byte loop */
	for (i = 0; i < len; i++) {
		u8 tx_byte = tx_buf ? tx_buf[i] : 0xFF;
		u8 rx_byte;

		ret = mindgrove_spi_wait_tx_not_full(spi);
		if (ret)
			goto out_disable;

		writeb(tx_byte, spi->base + MINDGROVE_SPI_REG_TX);

		ret = mindgrove_spi_wait_rx_not_empty(spi);
		if (ret)
			goto out_disable;

		rx_byte = readb(spi->base + MINDGROVE_SPI_REG_RX);
		if (rx_buf)
			rx_buf[i] = rx_byte;

		if (i < 8)
			dev_dbg(spi->dev,
				 "  byte[%u]: TX=0x%02x RX=0x%02x FIFO=0x%08x\n",
				 i, tx_byte, rx_byte,
				 readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS));
	}

	/* Step 3: wait for shift register idle */
	ret = mindgrove_spi_wait_complete(spi);

out_disable:
	/*
	 * Step 4: disable engine — clear EN only, keep NCS_OUTEN/SCLK_OUTEN/
	 * MOSI_OUTEN so that the subsequent set_cs(false) from the framework
	 * physically drives the NCS pin.
	 */
	mindgrove_spi_ctrl_disable(spi);

	dev_dbg(spi->dev,
		 "xfer END: ret=%d CTRL=0x%08x NCS_CTRL=0x%08x\n",
		 ret,
		 readl(spi->base + MINDGROVE_SPI_REG_CTRL),
		 readl(spi->base + MINDGROVE_SPI_REG_NCS_CTRL));

	return ret;
}

/* ------------------------------------------------------------------ */
/* Hardware initialisation                                             */
/* ------------------------------------------------------------------ */

static int mindgrove_spi_init_hw(struct mindgrove_spi *spi)
{
	unsigned long flags;

	spin_lock_irqsave(&spi->lock, flags);

	spi->num_cs = MINDGROVE_SPI_MAX_CS;

	/*
	 * U-Boot may leave the engine running (EN=1) with stale data in
	 * the FIFOs and COMM_STATUS showing busy.  Drain any leftover RX
	 * bytes first, then wait for the engine to go idle before resetting,
	 * otherwise the RX_FLUSH pulse may be ignored by a busy engine.
	 */
	{
		u32 timeout = 1000;

		/* Wait for engine idle */
		while ((readw(spi->base + MINDGROVE_SPI_REG_COMM_STATUS) &
			MINDGROVE_SPI_COMM_STATUS_BUSY) && --timeout)
			udelay(1);

		/* Drain any stale RX bytes */
		while (!(readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS) &
			 MINDGROVE_SPI_FIFO_STATUS_RX_EMPTY))
			readb(spi->base + MINDGROVE_SPI_REG_RX);
	}

	/*
	 * Temporary writes of 0 are only used during the flush sequence.
	 * After this block, CTRL must never be written as 0 again.
	 */

	/* Disable engine */
	writel(0, spi->base + MINDGROVE_SPI_REG_CTRL);
	udelay(10);
	/* Flush RX FIFO */
	writel(MINDGROVE_SPI_CTRL_RX_FLUSH(1), spi->base + MINDGROVE_SPI_REG_CTRL);
	udelay(10);
	writel(0, spi->base + MINDGROVE_SPI_REG_CTRL);
	/* Master mode (SLAVE_MODE=0 is a no-op write of 0, but matches U-Boot) */
	writel(MINDGROVE_SPI_CTRL_SLAVE_MODE(0), spi->base + MINDGROVE_SPI_REG_CTRL);

	/*
	 * CLK_CTRL: setup=1, hold=1, prescaler=INIT_PRESCALER.
	 * Full write clears any stale prescaler left by BBL/U-Boot.
	 */
	writel((MINDGROVE_SPI_INIT_PRESCALER << MINDGROVE_SPI_CLK_CTRL_PRESCALAR_SHIFT) |
	       (1u << MINDGROVE_SPI_CLK_CTRL_SETUP_SHIFT) |
	       (1u << MINDGROVE_SPI_CLK_CTRL_HOLD_SHIFT),
	       spi->base + MINDGROVE_SPI_REG_CLK_CTRL);

	spi->spi_freq  = spi->input_clk_hz / (MINDGROVE_SPI_INIT_PRESCALER + 1);
	spi->prescaler = MINDGROVE_SPI_INIT_PRESCALER;

	/*
	 * NCS_CTRL: CS0 selected (SELECT=1), SW=1 (NCS HIGH = deasserted).
	 * set_cs() will drive SW=0 when the first message starts.
	 */
	writel(MINDGROVE_SPI_NCS_CTRL_SELECT(1) | MINDGROVE_SPI_NCS_CTRL_SW(1),
	       spi->base + MINDGROVE_SPI_REG_NCS_CTRL);

	/*
	 * CRITICAL: set output-enable bits permanently.
	 *
	 * NCS_OUTEN must be set for NCS_CTRL.SW to physically drive the
	 * CS pin.  We keep all three output-enables on at all times and
	 * only toggle the EN bit per-transfer.
	 */
	writel(MINDGROVE_SPI_CTRL_PIN_MASK, spi->base + MINDGROVE_SPI_REG_CTRL);

	spin_unlock_irqrestore(&spi->lock, flags);

	dev_dbg(spi->dev,
		 "init_hw done:\n"
		 "  CTRL=0x%08x (expect 0x%08x)\n"
		 "  CLK_CTRL=0x%08x\n"
		 "  NCS_CTRL=0x%08x (expect 0x00000003)\n"
		 "  FIFO_STATUS=0x%08x COMM_STATUS=0x%04x\n",
		 readl(spi->base + MINDGROVE_SPI_REG_CTRL),
		 MINDGROVE_SPI_CTRL_PIN_MASK,
		 readl(spi->base + MINDGROVE_SPI_REG_CLK_CTRL),
		 readl(spi->base + MINDGROVE_SPI_REG_NCS_CTRL),
		 readl(spi->base + MINDGROVE_SPI_REG_FIFO_STATUS),
		 readw(spi->base + MINDGROVE_SPI_REG_COMM_STATUS));

	return 0;
}

/* ------------------------------------------------------------------ */
/* Platform driver probe / remove                                       */
/* ------------------------------------------------------------------ */

static int mindgrove_spi_probe(struct platform_device *pdev)
{
	struct spi_controller *ctlr;
	struct mindgrove_spi *spi;
	struct resource *res;
	int ret;

	ctlr = spi_alloc_host(&pdev->dev, sizeof(struct mindgrove_spi));
	if (!ctlr) {
		dev_err(&pdev->dev, "SPI controller allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, ctlr);
	spi = spi_controller_get_devdata(ctlr);
	spi->dev  = &pdev->dev;
	spi->ctlr = ctlr;
	spin_lock_init(&spi->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No MEM resource\n");
		ret = -ENODEV;
		goto put_ctlr;
	}
	spi->phys = res->start;

	spi->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(spi->base)) {
		ret = PTR_ERR(spi->base);
		dev_err(&pdev->dev, "ioremap failed: %d\n", ret);
		goto put_ctlr;
	}

	dev_dbg(&pdev->dev, "SPI registers: phys=%pa virt=%px\n",
		 &spi->phys, spi->base);

	if (of_property_read_u32(pdev->dev.of_node,
				 "mindgrove,fifo-depth", &spi->fifo_depth))
		spi->fifo_depth = MINDGROVE_SPI_DEFAULT_DEPTH;

	if (of_property_read_u32(pdev->dev.of_node,
				 "mindgrove,max-bits-per-word",
				 &spi->bits_per_word))
		spi->bits_per_word = MINDGROVE_SPI_DEFAULT_BITS;

	//spi->lsb_first = of_property_read_bool(pdev->dev.of_node,
					       //"mindgrove,lsb-first");
	//spi->cpha = of_property_read_bool(pdev->dev.of_node, "mindgrove,cpha");
	//spi->cpol = of_property_read_bool(pdev->dev.of_node, "mindgrove,cpol");

		// Temporary variables to safely read the 32-bit integers from DTS
	u32 val_lsb = 0, val_cpha = 0, val_cpol = 0;

	if (!of_property_read_u32(pdev->dev.of_node, "mindgrove,lsb-first", &val_lsb))
		spi->lsb_first = (val_lsb != 0);
	else
		spi->lsb_first = false;

	if (!of_property_read_u32(pdev->dev.of_node, "mindgrove,cpha", &val_cpha))
		spi->cpha = (val_cpha != 0);
	else
		spi->cpha = false;

	if (!of_property_read_u32(pdev->dev.of_node, "mindgrove,cpol", &val_cpol))
		spi->cpol = (val_cpol != 0);
	else
		spi->cpol = false;

	spi->input_clk_hz = 50000000;	/* 50 MHz, matches U-Boot */

	if (of_property_read_u32(pdev->dev.of_node,
				 "spi-max-frequency", &spi->spi_freq))
		spi->spi_freq = MINDGROVE_SPI_MAX_FREQ;

	ret = mindgrove_spi_init_hw(spi);
	if (ret)
		goto put_ctlr;

	ctlr->dev.of_node	    = pdev->dev.of_node;
	ctlr->bus_num		    = pdev->id;
	ctlr->num_chipselect	    = MINDGROVE_SPI_MAX_CS;
	ctlr->bits_per_word_mask    = SPI_BPW_MASK(spi->bits_per_word);
	ctlr->max_speed_hz	    = spi->input_clk_hz / 2;
	ctlr->min_speed_hz	    = spi->input_clk_hz / (0x3FFF + 1);
	ctlr->mode_bits		    = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST | SPI_CS_HIGH;

	ctlr->setup		    = mindgrove_spi_setup;
	ctlr->prepare_message	    = mindgrove_spi_prepare_msg;
	ctlr->unprepare_message	    = mindgrove_spi_unprepare_msg;
	ctlr->transfer_one	    = mindgrove_spi_transfer_one;
	ctlr->set_cs		    = mindgrove_spi_set_cs;

	ret = spi_register_controller(ctlr);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register SPI controller: %d\n", ret);
		goto put_ctlr;
	}

	dev_dbg(&pdev->dev, "Mindgrove SPI registered (clk=%u Hz, fifo=%u)\n",
		 spi->input_clk_hz, spi->fifo_depth);
	return 0;

put_ctlr:
	spi_controller_put(ctlr);
	return ret;
}

static void mindgrove_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *ctlr = platform_get_drvdata(pdev);
	struct mindgrove_spi *spi = spi_controller_get_devdata(ctlr);

	/* Full disable on remove is fine since the device is going away */
	writel(0, spi->base + MINDGROVE_SPI_REG_CTRL);
	spi_unregister_controller(ctlr);
}

static const struct of_device_id mindgrove_spi_of_match[] = {
	{ .compatible = "mindgrove,spi" },
	{}
};
MODULE_DEVICE_TABLE(of, mindgrove_spi_of_match);

static struct platform_driver mindgrove_spi_driver = {
	.probe  = mindgrove_spi_probe,
	.remove = mindgrove_spi_remove,
	.driver = {
		.name		= MINDGROVE_SPI_DRIVER_NAME,
		.of_match_table	= mindgrove_spi_of_match,
	},
};
module_platform_driver(mindgrove_spi_driver);

MODULE_ALIAS("platform:" MINDGROVE_SPI_DRIVER_NAME);
MODULE_AUTHOR("Evil Gorg");
MODULE_DESCRIPTION("Mindgrove SPI controller driver");
MODULE_LICENSE("GPL v2");