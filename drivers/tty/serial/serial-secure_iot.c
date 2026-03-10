

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

struct secure_iot_serial_port{
    struct uart_port port;
    struct device *dev;
    unsigned char ier;
    unsigned long baud_rate;
    struct clk *clk;
};

#define SECURE_IOT_MAX_UART 4

struct platform_device * mindgrove_serial_ports[SECURE_IOT_MAX_UART];

#define BAUD_REG_OFS	0x00	//16-bit	Baud rate divisor
#define RESERVED	0x02	//16-bit	Padding
#define TX_REG	0x04	//32-bit*	Transmit Data (Assuming Data is 32-bit)
#define RX_REG	0x08	//32-bit*	Receive Data (Assuming Data is 32-bit)
#define STATUS_REG	0x0C	//16-bit	Status Flags (Parity, Errors, FIFO state)
#define RESERVED1	0x0E	//16-bit	Padding
#define DELAY_REG	0x10	//16-bit	Inter-transmission delay
#define RESERVED2	0x12	//16-bit	Padding
#define CTRL	0x14	//16-bit	Config (Stop bits, Parity, Char size)
#define RESERVED3	0x16	//16-bit	Padding
#define INTR_EN	0x18	//16-bit	Interrupt Masking
#define RESERVED4	0x1A	//16-bit	Padding
#define RX_THRESHOLD	0x1C	//8-bit	FIFO Almost Full trigger level
#define RESERVED5	0x1D	//8-bit	Padding
#define RESERVED6	0x1E	//16-bit	Final Padding to reach 0x20

#define TX_DATA_EMPTY_SHIFT 0
#define TX_DATA_EMPTY_MASK (1 << TX_DATA_EMPTY_SHIFT)
#define TX_DATA_FULL_SHIFT 1
#define TX_DATA_FULL_MASK (1 << TX_DATA_FULL_SHIFT)
#define RX_NOT_EMPTY_SHIFT 2
#define RX_NOT_EMPTY_MASK (1 << RX_NOT_EMPTY_SHIFT)
#define RX_FULL_SHIFT 3

static int __init mindgrove_serial_init(void){
    uart_register_driver(&mindgrove_serial);
    //for registering the struct specific to serial datatype.
    platform_driver_register(&mindgrove_serial_driver);
    return 0;
}

static void __exit mindgrove_serial_exit(void){
    platform_driver_unregister(&mindgrove_serial_driver);
    uart_unregister_driver(&mindgrove_serial);
}

module_init(mindgrove_serial_init);
module_exit(mindgrove_serial_exit);

static void mindgrove_uart_putc(struct uart_port *port ,unsigned char c){
    while(__raw_readl(port->membase+STATUS_REG)&TX_DATA_FULL_MASK){
        cpu_relax();
    }
    __raw_writel(c,port->membase+TX_REG);
}

static bool uart_rx_empty(struct uart_port *port){
    return (!(__raw_readl(port->membase+STATUS_REG)&RX_NOT_EMPTY_MASK));
}

static char uart_getc(struct uart_port *port){
    while(uart_rx_empty(port)){
        cpu_relax();
    }
    return (char)(__raw_readl(port->membase+RX_REG));
}

static void mindgrove_uart_start_tx(struct uart_port *port){
    struct circ_buf *xmit = &port->state->xmit;
    while(!uart_circ_empty(xmit)){
        mindgrove_uart_putc(port,xmit->buf[xmit->tail]);
        xmit->tail = ((xmit->tail+1) &  (UART_XMIT_SIZE - 1));
        port->icount.tx++;
    }
}

// static char mindgrove_recieve_char(struct uart_port *port,char *is_empty){
//     u32 v;
//     u8 ch;
//     v = __mindgrove_readl(port,RX_REG);
//     if(! is_empty){
//         WARN_ON(1);
//     }
//     else{
//         *is_empty = ~ ((v & RX_NOT_EMPTY_MASK) >> RX_NOT_EMPTY_SHIFT);
//     }
//     ch = (v & RX_REG) 
// }
//writeel intended to be used only in the early boot code

static void __mindgrove_early_writel(u32 v ,u16 offs, struct uart_port *port)
{
    writel_relaxed(v ,port->membase + offs);
    //Is this specifif to sifive code or just relaxed?
}

static u32 __mindgrove_early_readl(struct uart_port *port, u16 offs){
    return readl_relaxed(port->membase + offs);
}

static void __mindgrove_writel(u32 v ,u16 offs, struct secure_iot_serial_port *ssp)
{
    __mindgrove_early_writel(v,offs,&ssp->port);
    //pointing to the generic uarttype
}

static u32 __mindgrove_readl(struct secure_iot_serial_port *ssp ,u16 offs)
{
    return __mindgrove_early_readl(&ssp->port,offs);
}

static int mindgrove_serial_is_txfifo_full(struct secure_iot_serial_port *ssp)
{
    return __mindgrove_readl(ssp, STATUS_REG) & TX_DATA_FULL_MASK;
}

static int mindgrove_serial_is_txfifo_empty(struct secure_iot_serial_port *ssp)
{
    return __mindgrove_readl(ssp,STATUS_REG) & TX_DATA_EMPTY_MASK;
}

static int mindgrove_serial_get_mctrl(struct uart_port *port)
{
    return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR;
}

static void __mindgre_transmit_char(struct secure_iot_serial_port *ssp ,int ch)
{
    __mindgrove_writel(ch , TX_REG ,ssp);
}

static char __mindgrove_recieve_data(struct secure_iot_serial_port *ssp,char *is_empty){
    u32 status;
    u32 data_reg;
    u8 ch;
    // v = __mindgrove_readl(ssp,RX_REG);
    // if (! is_empty)
    //     WARN_ON(1);
    // else{

    // }
    status = readw(ssp->membase + STATUS_REG);
    if(!is_empty){
        WARN_ON(1);
    }
    else{
        *is_empty = !(status & RX_NOT_EMPTY_MASK);
    }
    //If data is available read the data register
    if(status & RX_NOT_EMPTY_MASK){
        data_reg = readl(ssp->membase+RX_REG);
        ch = (u8) (data_reg & 0xFF);
    }
    return ch;
}

static void mindgrove_init_port(struct uart_port *mindgrove_port, struct platform_device *pdev){
    //Initiallization of the port struct at the start of the nitialization code in the script.
    struct uart_port *port =&mindgrove_port->port;
    struct mindgrove_uart_data *data =pdev->dev.platform_data;

    port->iotype = UPIO_MEM;
    port->flags = UPF_BOOT_AUTOCONF;
    port->ops = &mindgrove_ops;
    port->fifosize = 1;
    port->line = pdev->id;
    port->dev = &pdev->dev;
    port->mapbase =pdev->resource[0].start;
    port->membase=NULL;
    //initializing the ring buffer to 0 values at the start.
    memset(&mindgrove_port->rx_ring, 0, sizeof(mindgrove_port->rx_ring));

    port->irq =pdev->resource[1].start;
    tasklet_init(&mindgrove_port->tasklet,mindgrove_task_func,(unsigned long)port);

    if(data->regs){
        port->membase = data->regs;
    }
    else{
        port->flags |= UPF_IOREMAP;
        port->membase =NULL;
    }
    // for the console the clk could already be set
    if(!mindgrove_port->clk){
        mindgrove_port->clk = clk_get(&pdev->dev,"usart");
        clk_enable(mindgrove_port->clk);
        port->uartclk = clk_get_rate(mindgrove_port->clk);
        clk_disable(mindgrove_port->clk);
    }
}

static int mindgrove_serial_probe(platform_device *pdev)
{
	struct altera_jtaguart_platform_uart *platp =
			dev_get_platdata(&pdev->dev);
	struct uart_port *port;
	struct resource *res_mem;
	int i = pdev->id;
	int irq;

	/* -1 emphasizes that the platform must have one port, no .N suffix */
	if (i == -1)
		i = 0;

	if (i >= ALTERA_JTAGUART_MAXPORTS)
		return -EINVAL;

	port = &altera_jtaguart_ports[i];

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res_mem)
		port->mapbase = res_mem->start;
	else if (platp)
		port->mapbase = platp->mapbase;
	else
		return -ENODEV;

	irq = platform_get_irq_optional(pdev, 0);
	if (irq < 0 && irq != -ENXIO)
		return irq;
	if (irq > 0)
		port->irq = irq;
	else if (platp)
		port->irq = platp->irq;
	else
		return -ENODEV;

	port->membase = ioremap(port->mapbase, ALTERA_JTAGUART_SIZE);
	if (!port->membase)
		return -ENOMEM;

	port->line = i;
	port->type = PORT_ALTERA_JTAGUART;
	port->iotype = SERIAL_IO_MEM;
	port->ops = &altera_jtaguart_ops;
	port->flags = UPF_BOOT_AUTOCONF;
	port->dev = &pdev->dev;

	uart_add_one_port(&altera_jtaguart_driver, port);

	return 0;
}

static void mindgrove_serial_remove(struct platform_device *pdev)
{
    struct uart_port *port;
    int i= pdev->id;
    if(i==-1){
        i=0;
    }
    port =&mindgrove_serial_ports[i];
    //port->backup_imr =0;
    //How I am to store the various instances of the uart struct.
    uart_remove_one_port(&mindgrove_uart,port);
    iounmap(port->membase);
}

static unsigned int secure_get_mctrl(struct uart_port *port)
{
    /* Always ready: no real inputs, simulate active CTS/DCD/DSR */
    return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR;
    /* Exclude TIOCM_RI */
}

static void secure_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
    /* Track software state if needed (core already does) */
    port->mctrl = mctrl;
    /* No-op: no registers/GPIOs; loopback ignored */
    /* Optional: dev_dbg(port->dev, "MCTRL: 0x%x\n", mctrl); */
}


static const struct uart_ops mindgrove_pops={
    .type = mindgrove_uart_type,
    .tx_empty=mindgrove_tx_empty,
    .set_mctrl=mindgrove_serial_set_mctrl,
    .get_mctrl=mindgrove_serial_get_mctrl,
    .set_terminos=mindgrove_serial_set_terminos,
    .stop_tx=mindgrove_uart_stop_tx,
    .start_tx=mindgrove_uart_start_tx,
    .stop_rx=mindgrove_uart_stop_rx,
    .break_ctrl = mindgrove_serial_break_ctrl,
    .startup=mindgrove_serial_startup,
    .shutdown=mindgrove_serial_shutdown,

}

static struct uart_driver mindgrove_uart = {
    .owner=THIS_MODULE,
    .driver_name ="mindgrove_serial",
    .dev_name = MINDGROVE_DEVICENAME,
    .major = SERIAL_MAJOR,
    .minor = MINOR_START,
    .nr = SECURE_IOT_MAX_UART,
    .cons = MINDGROVE_CONSOLE_DEVICE,
};

static const struct of_device_id mindgrove_serial_of_match[] = {
    { .compatible = "mindgrove", "secure_iot_uart"},
    {},
};

MODULE_DEVICE_TABLE(of,mindgrove_serial_of_match);

static struct platform_driver mindgrove_serial_driver{
    .probe = mindgrove_serial_probe,
    .remove = mindgrove_serial_remove,
    .suspend = mindgrove_serial_suspend,
    .resume = mindgrove_serial_resume,
    .driver = {
        .name = "mindgrove_uart",
        .owner = THIS_MODULE,
        .of_match_table = mindgrove_serial_of_match,
    },
};

//FOR selecting the output console device -> to display the output.
MODULE_DESCRIPTION("Mindgrove Secure_IoT serial drivers");
MODULE_AUTHOR("Biancaa Ramesh <biancaa2210329@ssn.edu.in>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" KBUILD_MODNAME);MODULE_LICENSE("GPL");

