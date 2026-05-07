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
    void __iomem *reg;
    unsigned char console_line_ended;
};

#define SECURE_IOT_MAX_UART 4
#define MINDGROVE_CONSOLE_DEVICE NULL
#define PORT_SECURE_IOT_V0 123

struct platform_device * mindgrove_serial_ports[SECURE_IOT_MAX_UART];

#define BAUD_REG	0x00	//16-bit	Baud rate divisor
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
#define RX_FULL_MASK (1<<RX_FULL_SHIFT)
#define RX_ALMOST_FULL_SHIFT 8
#define RX_ALMOST_FULL_MASK (1<<8)
#define SECURE_IOT_RX_FIFO_DEPTH 32
#define SERIAL_MAJOR TTY_MAJOR
#define SERIAL_MINOR 64
#define MINDGROVE_DEVICENAME "ttyMG0"

#define port_to_mindgrove_serial_port(p) \
    container_of((p), struct secure_iot_serial_port, port)

#define MINDGROVE_SERIAL_IE_TX_EMPTY (1 << 0)
#define MINDGROVE_SERIAL_IE_RX_NOT_EMPTY (1 << 2)
#define MINDGROVE_SERIAL_STOP_BIT_SHIFT 1
#define MINDGROVE_SERIAL_STOP_BIT_MASK (3 << MINDGROVE_SERIAL_STOP_BIT_SHIFT)
#define MINDGROVE_SERIAL_IE_TXWM_MASK (3<<0)
#define MINDGROVE_SERIAL_IE_RXWM_MASK (67<<3)

/* Defined at the top to avoid 'undeclared' errors */
static struct uart_driver mindgrove_uart;
static struct platform_driver mindgrove_serial_driver;
static const struct uart_ops mindgrove_pops;

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

// static void mindgrove_uart_start_tx(struct uart_port *port){
//     struct circ_buf *xmit = &port->state->xmit;
//     while(!uart_circ_empty(xmit)){
//         mindgrove_uart_putc(port,xmit->buf[xmit->tail]);
//         xmit->tail = ((xmit->tail+1) &  (UART_XMIT_SIZE - 1));
//         port->icount.tx++;
//     }
// }

static void mindgrove_uart_start_tx(struct uart_port *port) {
    struct secure_iot_serial_port *ssp = port_to_mindgrove_serial_port(port);
    struct circ_buf *xmit = &port->state->xmit;

    while (!uart_circ_empty(xmit)) {
        // Check if hardware is full. If it is, STOP and wait for the next IRQ.
        if (readl(ssp->reg + STATUS_REG) & TX_DATA_FULL_MASK)
            break;

        writel(xmit->buf[xmit->tail], ssp->reg + TX_REG);
        xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
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

static __u32 __mindgrove_readl(struct secure_iot_serial_port *ssp ,u16 offs)
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

static void __mindgrove_transmit_char(struct secure_iot_serial_port *ssp ,int ch)
{
    __mindgrove_writel(ch , TX_REG ,ssp);
}

static char __mindgrove_receive_char(struct secure_iot_serial_port *ssp,char *is_empty){
    u32 status;
    u32 data_reg;
    u8 ch=0;
    // v = __mindgrove_readl(ssp,RX_REG);
    // if (! is_empty)
    //     WARN_ON(1);
    // else{

    // }
    status = readw(ssp->reg + STATUS_REG);
    if(!is_empty){
        WARN_ON(1);
    }
    else{
        *is_empty = !(status & RX_NOT_EMPTY_MASK);
    }
    //If data is available read the data register
    if(status & RX_NOT_EMPTY_MASK){
        data_reg = readl(ssp->reg+RX_REG);
        ch = (u8) (data_reg & 0xFF);
    }
    return ch;
}

static void mindgrove_receive_chars(struct secure_iot_serial_port *ssp)
{
    char is_empty;
    int c;
    u8 ch;

    for(c =SECURE_IOT_RX_FIFO_DEPTH; c>0; c--){
        ch = __mindgrove_receive_char(ssp, &is_empty);
        if(is_empty){
            break;
        }
        ssp->port.icount.rx++;
        if(!uart_prepare_sysrq_char(&ssp->port,ch)){
            uart_insert_char(&ssp->port,0,0,ch,TTY_NORMAL);
        }
    }

    tty_flip_buffer_push(&ssp->port.state->port);
}

static void mindgrove_serial_set_stop_bits(struct secure_iot_serial_port *ssp, char nstop)
{
	u16 ctrl_reg;

	if (nstop < 1 || nstop > 2) {
		WARN_ON(1);
		return;
	}

	ctrl_reg = readw(ssp->reg+CTRL);
    //AS it is a 16 bit register in this case.
	ctrl_reg &= ~MINDGROVE_SERIAL_STOP_BIT_MASK;
	ctrl_reg |= (nstop - 1) << MINDGROVE_SERIAL_STOP_BIT_SHIFT;
	writew(ctrl_reg,ssp->reg+CTRL);
}

// static void mindgrove_update_div(struct secure_iot_serial_port *ssp)
// {
//     u16 div;
//     div = DIV_ROUND_UP(ssp->port.uartclk,ssp->baud_rate) -1;
//     writel(div, );
//     //prescalar update?

// }

static void __mindgrove_enable_txwm(struct secure_iot_serial_port *ssp)
{
    if(ssp->ier & MINDGROVE_SERIAL_IE_TXWM_MASK){
        return;
    }
    ssp->ier |= MINDGROVE_SERIAL_IE_TXWM_MASK;
    u32 intr_en_ctrl = readl(ssp->reg+INTR_EN);
    intr_en_ctrl |= ssp->ier;
    writel(intr_en_ctrl, ssp->reg+INTR_EN);
}

static void __mindgrove_enable_rxwm(struct secure_iot_serial_port *ssp)
{
    if(ssp->ier & MINDGROVE_SERIAL_IE_RXWM_MASK){
        return;
    }
    ssp->ier |= MINDGROVE_SERIAL_IE_RXWM_MASK;
    u32 intr_en_ctrl = readl(ssp->reg+INTR_EN);
    intr_en_ctrl |= ssp->ier;
    writel(intr_en_ctrl, ssp->reg+INTR_EN);
}

static void __mindgrove_disable_txwm(struct secure_iot_serial_port *ssp)
{
    if(!(ssp->ier&MINDGROVE_SERIAL_IE_TXWM_MASK)){
        return;
    }
    ssp->ier &= ~MINDGROVE_SERIAL_IE_TXWM_MASK;
    u32 intr_en_ctrl = readl(ssp->reg+INTR_EN);
    intr_en_ctrl &= ssp->ier; //already cleared and 
    writel(intr_en_ctrl, ssp->reg+INTR_EN);    
}

static void __mindgrove_disable_rxwm(struct secure_iot_serial_port * ssp)
{
    if(!(ssp->ier&MINDGROVE_SERIAL_IE_RXWM_MASK)){
        return;
    }
    ssp->ier &= ~MINDGROVE_SERIAL_IE_RXWM_MASK;
    u32 intr_en_ctrl = readl(ssp->reg+INTR_EN);
    intr_en_ctrl &= ssp->ier;
    writel(intr_en_ctrl, ssp->reg+INTR_EN);   

}

static void mindgrove_set_delay(struct secure_iot_serial_port *ssp, u16 delay){
    writel(delay , ssp->reg+DELAY_REG);
}

static void mindgrove_update_baud_rate(struct secure_iot_serial_port *ssp , unsigned int rate)
{
    if (ssp->baud_rate == rate){
        return;
    }
    ssp->baud_rate = rate;
    writel(rate, ssp->reg+BAUD_REG);
}

// wait for xmitr function ---> not referenced by a callback any where .
static void __maybe_unused __secure_wait_for_xmitr(struct secure_iot_serial_port *ssp)
{
    while(mindgrove_serial_is_txfifo_full(ssp)){
        udelay(1);
    }
}

/* linux serial API functions to start/stop tx ,rx functionality. */

static void secure_stop_tx(struct uart_port *port)
{
    struct secure_iot_serial_port *ssp = port_to_mindgrove_serial_port(port);
    __mindgrove_disable_txwm(ssp);
}

static void secure_serial_stop_rx(struct uart_port *port)
{
    struct secure_iot_serial_port *ssp = port_to_mindgrove_serial_port(port);
    __mindgrove_disable_rxwm(ssp);
    __mindgrove_enable_txwm(ssp);
}

static void secure_serial_start_rx(struct uart_port *port)
{
    struct secure_iot_serial_port *ssp = port_to_mindgrove_serial_port(port);
    __mindgrove_enable_rxwm(ssp);
}

static struct platform_driver mindgrove_serial_driver = {
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

static void mindgrove_init_port(struct uart_port *mindgrove_port, struct platform_device *pdev, struct secure_iot_serial_port *ssp){
    //Initiallization of the port struct at the start of the nitialization code in the script.
    struct uart_port *port =&mindgrove_port->port;
    struct mindgrove_uart_data *data =pdev->dev.platform_data;

    port->iotype = UPIO_MEM;
    port->iobase = pdev->resource[0]; //0x11300
    port->flags = UPF_BOOT_AUTOCONF;
    //port->ops = &mindgrove_ops;
    port->ops = &mindgrove_pops;
    port->fifosize = SECURE_IOT_RX_FIFO_DEPTH;
    port->line = pdev->id;
    port->dev = &pdev->dev;
    port->mapbase =pdev->resource[0].start;
    port->membase=NULL;
    //initializing the ring buffer to 0 values at the start.
    //memset(&mindgrove_port->rx_ring, 0, sizeof(mindgrove_port->rx_ring));

    port->irq =pdev->resource[1].start;
    //tasklet_init(&mindgrove_port->tasklet,mindgrove_task_func,(unsigned long)port);

    if(data->regs){
        port->membase = data->regs;
    }
    else{
        port->flags |= UPF_IOREMAP;
        port->membase =NULL;
    }
    // for the console the clk could already be set
    if(!ssp->clk){
        // mindgrove_port->clk = clk_get(&pdev->dev,"usart");
        // clk_enable(mindgrove_port->clk);
        // port->uartclk = clk_get_rate(mindgrove_port->clk);
        // clk_disable(mindgrove_port->clk);
        if (!IS_ERR(ssp->clk)) {
            clk_prepare_enable(ssp->clk);
            port->uartclk = clk_get_rate(ssp->clk);
        } else {
            port->uartclk = 16000000; // Fallback or handle error
        }
    }
}

static irqreturn_t mindgrove_serial_irq(int irq, void *dev_id)
{
    struct secure_iot_serial_port *ssp = dev_id;
    u32 status;

    status = readl(ssp->reg + STATUS_REG);

    if (status & RX_NOT_EMPTY_MASK)
        mindgrove_receive_chars(ssp);

    if (status & TX_DATA_EMPTY_MASK)
        mindgrove_uart_start_tx(&ssp->port);

    return IRQ_HANDLED;
}

static int mindgrove_serial_probe(struct platform_device *pdev)
{
    struct secure_iot_serial_port *ssp;
    struct uart_port *port;
    struct resource *res;
    int ret;

    ssp = devm_kzalloc(&pdev->dev, sizeof(*ssp), GFP_KERNEL);
    if (!ssp)
        return -ENOMEM;

    port = &ssp->port;
    mindgrove_init_port(port,pdev,ssp); // Use your helper!
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    port->mapbase = res->start;
    port->membase = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(port->membase))
        return PTR_ERR(port->membase);

    ssp->reg = port->membase; // Initialize the ssp->reg pointer
    port->irq = platform_get_irq(pdev, 0);
    port->dev = &pdev->dev;
    port->type = PORT_SECURE_IOT_V0;
    port->flags= UPF_BOOT_AUTOCONF;
    port->line     = pdev->id >= 0 ? pdev->id : 0;
    port->ops = &mindgrove_pops; // Use your ops struct
    port->fifosize = 16; // Adjust based on your HW specs

    platform_set_drvdata(pdev, ssp);

    ret = uart_add_one_port(&mindgrove_uart, port);
    if (ret)
        return ret;

    return devm_request_irq(&pdev->dev, port->irq, mindgrove_serial_irq, 
                           0, dev_name(&pdev->dev), ssp);
}

// static int mindgrove_serial_probe(platform_device *pdev)
// {
// 	struct altera_jtaguart_platform_uart *platp =
// 			dev_get_platdata(&pdev->dev);
// 	struct uart_port *port;
// 	struct resource *res_mem;
// 	int i = pdev->id;
// 	int irq;

// 	/* -1 emphasizes that the platform must have one port, no .N suffix */
// 	if (i == -1)
// 		i = 0;

// 	if (i >= ALTERA_JTAGUART_MAXPORTS)
// 		return -EINVAL;

// 	port = &altera_jtaguart_ports[i];

// 	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
// 	if (res_mem)
// 		port->mapbase = res_mem->start;
// 	else if (platp)
// 		port->mapbase = platp->mapbase;
// 	else
// 		return -ENODEV;

// 	irq = platform_get_irq_optional(pdev, 0);
// 	if (irq < 0 && irq != -ENXIO)
// 		return irq;
// 	if (irq > 0)
// 		port->irq = irq;
// 	else if (platp)
// 		port->irq = platp->irq;
// 	else
// 		return -ENODEV;

// 	port->membase = ioremap(port->mapbase, ALTERA_JTAGUART_SIZE);
// 	if (!port->membase)
// 		return -ENOMEM;

// 	port->line = i;
// 	port->type = PORT_ALTERA_JTAGUART;
// 	port->iotype = SERIAL_IO_MEM;
// 	port->ops = &altera_jtaguart_ops;
// 	port->flags = UPF_BOOT_AUTOCONF;
// 	port->dev = &pdev->dev;

// 	uart_add_one_port(&altera_jtaguart_driver, port);
//     // Inside probe
//     int ret = devm_request_irq(&pdev->dev, port->irq, mindgrove_serial_irq, 
//                        IRQF_SHARED, dev_name(&pdev->dev), ssp);
//     //Requesting of the irq that was mentioned in the dts file of the device .

// 	return 0;
// }

static void mindgrove_uart_stop_tx(struct uart_port *port)
{
    struct secure_iot_serial_port *ssp = port_to_mindgrove_serial_port(port);
    //disable tx interrupts mask
    // 16 bit interrupt enable register is considered.
    u16 interrupt_en = readw(ssp->reg + INTR_EN);
    interrupt_en &= ~(TX_DATA_EMPTY_MASK);
    interrupt_en &= ~(TX_DATA_FULL_MASK);
    writew(interrupt_en , ssp->reg+INTR_EN);
}

static void mindgrove_uart_stop_rx(struct uart_port *port)
{
    struct secure_iot_serial_port *ssp =port_to_mindgrove_serial_port(port);
    //disable rx interrupt masks
    // 16 bit interrupt enable register is considered.
    u16 interrupt_en = readw(ssp->reg+INTR_EN);
    interrupt_en &= ~(RX_NOT_EMPTY_MASK);
    interrupt_en &= ~(RX_FULL_MASK);
    interrupt_en &= ~(RX_ALMOST_FULL_MASK);
    writew(interrupt_en , ssp->reg+INTR_EN);
}

static int mindgrove_serial_startup(struct uart_port *port)
{
    struct secure_iot_serial_port *ssp =port_to_mindgrove_serial_port(port);
    unsigned long flags;
    // default functions 
    uart_port_lock_irqsave(&ssp->port, &flags);
    //enable all rx interrupts for reception.
    u16 interrupt_en = readl(ssp->reg+INTR_EN);
    interrupt_en |= (RX_NOT_EMPTY_MASK);
    interrupt_en |= (RX_FULL_MASK);
    interrupt_en |= (RX_ALMOST_FULL_MASK);
    writel(interrupt_en , ssp->reg+INTR_EN);
    uart_port_unlock_irqrestore(&ssp->port,flags);
    return 0;

}

static void mindgrove_serial_shutdown(struct uart_port *port)
{
    struct secure_iot_serial_port *ssp = port_to_mindgrove_serial_port(port);
    unsigned long flags;
    uart_port_lock_irqsave(&ssp->port ,&flags);
    //disable rx interrupts
    u16 interrupt_en = readl(ssp->reg+INTR_EN);
    interrupt_en &= ~(RX_NOT_EMPTY_MASK);
    interrupt_en &= ~(RX_FULL_MASK);
    interrupt_en &= ~(RX_ALMOST_FULL_MASK);
    interrupt_en &= ~(TX_DATA_EMPTY_MASK);
    interrupt_en &= ~(TX_DATA_FULL_MASK);
    writel(interrupt_en , ssp->reg+INTR_EN);
    uart_port_unlock_irqrestore(&ssp->port,flags);
}

static void mindgrove_serial_set_termios(struct uart_port *port, struct ktermios* termios ,const struct ktermios *old)
{
    struct secure_iot_serial_port *ssp = port_to_mindgrove_serial_port(port);
    unsigned long flags;
    u32 v,old_v;
    int rate;
    char nstop;
    
    if((termios->c_cflag &CSIZE) != CS8)
    {
        dev_err_once(ssp->port.dev, "Only 8 bit words are supported \n");
        termios->c_cflag &= ~CSIZE;
        termios->c_cflag |= CS8;
    }
    if (termios->c_iflag &(INPCK | PARMRK))
        dev_err_once(ssp->port.dev ,"parity checking not supported \n");
    if (termios->c_iflag &BRKINT)
        dev_err_once(ssp->port.dev,"Break detection not supported\n");
    termios->c_iflag &= ~(INPCK|PARMRK|BRKINT);

    //Setting the number of stop bits
    nstop = (termios->c_cflag & CSTOPB) ? 2 : 1;
    mindgrove_serial_set_stop_bits(ssp,nstop);
    /*Setting of the line rate in drivers*/
    rate = uart_get_baud_rate(port,termios,old,0,ssp->port.uartclk/16);
    mindgrove_update_baud_rate(ssp,rate);
    uart_port_lock_irqsave(&ssp->port, &flags);

    /*Update per port timeout*/
    uart_update_timeout(port,termios->c_cflag,rate);
    ssp->port.read_status_mask =0;

    /*Ignore all characters if CREAD is not set*/
    v = readl(ssp->reg + INTR_EN);
    old_v =v;
    if((termios->c_cflag & CREAD) ==0 )
    {
        v &= ~RX_NOT_EMPTY_MASK;
        v &= ~RX_ALMOST_FULL_MASK;
        v &= ~RX_FULL_MASK;
    }
    else{
        v |= RX_NOT_EMPTY_MASK;
        v |= RX_ALMOST_FULL_MASK;
        v |= RX_FULL_MASK;
    }
    if(v!=old_v)
        writel(v,ssp->reg+INTR_EN);
    uart_port_unlock_irqrestore(&ssp->port,flags);
        
}

// For now I am using implementation for single serial core -> Future multiple serial simultaneously.
// static void mindgrove_serial_remove(struct platform_device *pdev)
// {
//     struct uart_port *port;
//     int i= pdev->id;
//     if(i==-1){
//         i=0;
//     }
//     port =&mindgrove_serial_ports[i];
//     //port->backup_imr =0;
//     //How I am to store the various instances of the uart struct.
//     uart_remove_one_port(&mindgrove_uart,port);
//     iounmap(port->membase);
// }

static void mindgrove_serial_remove(struct platform_device *pdev) {
    struct secure_iot_serial_port *ssp = platform_get_drvdata(pdev);

    if (ssp) {
        uart_remove_one_port(&mindgrove_uart, &ssp->port);
        if (!IS_ERR(ssp->clk))
            clk_disable_unprepare(ssp->clk);
    }
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

static void secure_break_ctl(struct uart_port*port ,int break_state)
{
    // IP doesnt allow sending a break;
}

static unsigned int secure_tx_empty(struct uart_port *port)
{
    //return TIOCSER_TEMT;
    struct secure_iot_serial_port *ssp = port_to_mindgrove_serial_port(port);
    return mindgrove_serial_is_txfifo_empty(ssp) ? TIOCSER_TEMT:0;
}

/* Implementation of some useless functions .*/
static void mindgrove_serial_release_port(struct uart_port *port)
{
}

static int mindgrove_serial_request_port(struct uart_port *port)
{
    return 0;
}

static void mindgrove_serial_config_port(struct uart_port * port)
{
    struct secure_iot_serial_port *ssp = port_to_mindgrove_serial_port(port);
    ssp->port.type = PORT_SECURE_IOT_V0;
}

static int mindgrove_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
    return -EINVAL;
}

static const char* mindgrove_uart_type(struct uart_port* port)
{
    return port->type == PORT_SECURE_IOT_V0 ? "SecureIoT UART V0" :NULL;
}

/*Polling based interrupt less functions inside irqs and other places when irqs are not functional.*/
#ifdef CONFIG_CONSOLE_POLL
static int mindgrove_serial_poll_get_char(struct uart_port *port)
{
    struct secure_iot_serial_port *ssp = port_to_mindgrove_serial_port(port);
    char is_empty,ch;
    ch = __mindgrove_receive_char(ssp, &is_empty);
    if(is_empty){
        return NO_POLL_CHAR;
    }
    return ch;
}

static void mindgrove_serial_poll_put_char(struct uart_port *port, unsigned char c)
{
    struct secure_iot_serial_port *ssp = port_to_mindgrove_serial_port(port);
    secure_wait_for_xmitr(ssp);
    __mindgrove_transmit_char(ssp,c);
}
#endif /*CONFIG_CONSOLE_POLL*/

/*Early console support*/
#ifdef CONFIG_SERIAL_EARLYCON
static void early_mindgrove_serial_putc(struct uart_port *port,unsigned char ch)
{
    while(__mindgrove_early_readl(port,TX_REG))
    {
        cpu_relax();
    }
    __mindgrove_early_writel(ch,TX_REG,port);
}

static void early_mindgrove_serial_write(struct console *con, const char *s, unsigned int n)
{
    struct earlycon_device *dev = con->data;
    struct uart_port *port =&dev->port;
    uart_console_write(port,s,n,early_mindgrove_serial_putc);
}

// Init function to execute on the starting of the console functions.
static int __init early_mindgrove_serial_setup(struct earlycon_device *dev, const char* options)
{
    struct uart_port *port = &dev->port;
    if(!port->membase){
        return -ENODEV;
    }
    dev->con->write = early_mindgrove_serial_write;
    return 0;
}

OF_EARLYCON_DECLARE(mindgrove, "mindgrove,uart",early_mindgrove_serial_setup);
#endif /*CONFIG_SERIAL_EARLYCON*/

/* Linux console interface */
#ifdef CONFIG_SERIAL_SECURE_IOT_CONSOLE
static struct secure_iot_serial_port *mindgrove_serial_console_ports[SECURE_IOT_MAX_UART];

static void mindgrove_serial_console_putchar(struct uart_port *port, unsigned char ch)
{
    struct secure_iot_serial_port *ssp = port_to_mindgrove_serial_port(port);
    __secure_wait_for_xmitr(ssp);
    __secure_transmit_char(ssp,ch);
    ssp->console_line_ended = (ch == '\n');
}

static void mindgrove_serial_device_lock(struct console* con , unsigned long *flags)
{
    struct uart_port *up = &mindgrove_serial_console_ports[con->index]->port;
    __uart_port_lock_irqsave(up,flags);
}

static void mindgrove_serial_device_unlock(struct console *con, unsigned long flags)
{
    struct uart_port *up =&mindgrove_serial_console_ports[con->index]->port;
    __uart_port_unlock_irqrestore(up,flags);
}

static void mindgrove_serial_console_write_atomic(struct console *con, struct nbcon_write_context *wctxt)
{
    struct secure_iot_serial_port *ssp = mindgrove_serial_console_ports[con->index];
    struct uart_port *port = &ssp->port;
    unsigned int ier;

    if (!ssp){
        return;
    }
    if(!nbcon_enter_unsafe(wctxt)){
        return;
    }
    ier = readl(ssp->reg+INTR_EN);
    writel(0,ssp->reg+INTR_EN);
    if(!ssp->console_line_ended){
        uart_console_write(port, "\n",1,mindgrove_serial_console_putchar);
    }
    uart_console_write(port, wctxt->outbuf, wctxt->len, mindgrove_serial_console_putchar);
    writel(ier,ssp->reg+INTR_EN);
    nbcon_exit_unsafe(wctxt);
}

static void mindgrove_serial_console_write_thread(struct console *con , struct nbcon_write_context *wctxt)
{
    struct secure_iot_serial_port *ssp =mindgrove_serial_console_ports[con->index];
    struct uart_port *port = &ssp->port;
    unsigned int ier;
    if(!ssp){
        return;
    }
    if(!nbcon_enter_unsafe(wctxt)){
        return;
    }
    ier= readl(ssp->reg+INTR_EN);
    writel(0,ssp->reg+INTR_EN);

    if(nbcon_exit_unsafe(wctxt)){
        int len = READ_ONCE(wxtxt->len);

        for(int i=0;i<len;i++)
        {
            if(!nbcon_enter_unsafe(wctxt))
                break;
            uart_console_write(port, wctxt->outpuf+i, mindgrove_serial_console_putchar);
            if(!nbcon_exit_unsafe(wctxt))
                break;
        }

        while(!nbcon_enter_unsafe(wctxt)){
            nbcon_reacquire_nobuf(wctxt);
        }
        writel(ier, ssp->reg+INTR_EN);
    }
}
#endif

static const struct uart_ops mindgrove_pops={
    .type = mindgrove_uart_type,
    .tx_empty=secure_tx_empty,
    .set_mctrl=secure_set_mctrl,
    .get_mctrl=secure_get_mctrl,
    .set_termios=mindgrove_serial_set_termios,
    .stop_tx=mindgrove_uart_stop_tx,
    .start_tx=mindgrove_uart_start_tx,
    .stop_rx=mindgrove_uart_stop_rx,
    .break_ctl = secure_break_ctl,
    .startup=mindgrove_serial_startup,
    .shutdown=mindgrove_serial_shutdown,
    .release_port = mindgrove_serial_release_port,
    .request_port = mindgrove_serial_request_port,
    .config_port = mindgrove_serial_config_port,
    .verify_port = mindgrove_serial_verify_port,

};

static struct uart_driver mindgrove_uart = {
    .owner=THIS_MODULE,
    .driver_name ="mindgrove_serial",
    .dev_name = MINDGROVE_DEVICENAME,
    .major = SERIAL_MAJOR,
    .minor = SERIAL_MINOR,
    .nr = SECURE_IOT_MAX_UART,
    .cons = MINDGROVE_CONSOLE_DEVICE,
};


static int __init mindgrove_serial_init(void){
    uart_register_driver(&mindgrove_uart);
    //Should be the name of the uart driver struct.
    //for registering the struct specific to serial datatype.
    platform_driver_register(&mindgrove_serial_driver);
    return 0;
}

static void __exit mindgrove_serial_exit(void){
    platform_driver_unregister(&mindgrove_serial_driver);
    uart_unregister_driver(&mindgrove_uart);
}

module_init(mindgrove_serial_init);
module_exit(mindgrove_serial_exit);

/******************************************************* 
 * 
 * PM operations in uart
 * 
*********************************************************/
static int mindgrove_serial_suspend(struct device *dev)
{
    struct secure_iot_serial_port *ssp = dev_get_drvdata(dev);
    return uart_suspend_port (&mindgrove_uart , &ssp->port );
}

static int mindgrove_serial_resume(struct device *dev)
{
    struct secure_iot_serial_port *ssp = dev_get_drvdata(dev);
    return uart_resume_port(mindgrove_uart, &ssp->port);
}

static DEFINE_SIMPLE_DEV_PM_OPS(secure_iot_pm_ops, mindgrove_serial_suspend, mindgrove_serial_resume);


static const struct of_device_id mindgrove_serial_of_match[] = {
    { .compatible = "mindgrove,uart"},
    {},
};

MODULE_DEVICE_TABLE(of,mindgrove_serial_of_match);


//FOR selecting the output console device -> to display the output.
MODULE_DESCRIPTION("Mindgrove Secure_IoT serial drivers");
MODULE_AUTHOR("Biancaa Ramesh <biancaa2210329@ssn.edu.in>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" KBUILD_MODNAME);MODULE_LICENSE("GPL");

