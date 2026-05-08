#include <linux/clk.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/io.h>
#include <linux/log2.h>
#include <linux/wait.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/iopoll.h>
#include <linux/types.h>


#define SECURE_IOT_SPI_DRIVER_NAME       "secure_iot_spi"
#define MINDGROVE_SPI_MAX_CS 4
#define MINDGROVE_SPI_DEFAULT_DEPTH 32
#define MINDGROVE_SPI_DEFAULT_BITS 8
#define MINDGROVE_SPI_TIMEOUT_US 1000000
#define MINDGROVE_SPI_MAX_FREQ 35000000

#define NCS_ENABLE 1
#define NCS_DISABLE 0
#define SPI_CLK_PRESCALE(x)     (((x) & 0x3FFFU) << 2)
#define SPI_LSB_FIRST_SHIFT(x) ((x)<<0)
#define  SPI_LSB_FIRST 1

/* Register offsets */
#define MINDGROVE_SPI_REG_CTRL 0x00		   /* Control register */
#define MINDGROVE_SPI_REG_CLK_CTRL 0x04	   /* Clock control register */
#define MINDGROVE_SPI_REG_TX 0x08		   /* TX data register */
#define MINDGROVE_SPI_REG_RX 0x0C		   /* RX data register */
#define MINDGROVE_SPI_REG_INTR_EN 0x10	   /* Interrupt enable */
#define MINDGROVE_SPI_REG_FIFO_STATUS 0x14 /* FIFO status */
#define MINDGROVE_SPI_REG_COMM_STATUS 0x18 /* Communication status */
#define MINDGROVE_SPI_REG_NCS_CTRL 0x1C	   /* NCS control */

/* CTRL register bit definitions */
#define MINDGROVE_SPI_CTRL_SLAVE_MODE(x) ((x) << 0)
#define MINDGROVE_SPI_CTRL_MISO_MODE(x)((x) << 24)
#define MINDGROVE_SPI_CTRL_EN(x) ((x) << 1)
#define MINDGROVE_SPI_CTRL_LSBFIRST(x) ((x) << 2)
#define MINDGROVE_SPI_CTRL_RX_FLUSH(x) ((x) << 3)
#define MINDGROVE_SPI_CTRL_COMM_MODE_MASK GENMASK(5, 4)
#define MINDGROVE_SPI_CTRL_COMM_MODE(x) ((x) << 4)
#define MINDGROVE_SPI_CTRL_TOTAL_BIT_TX_MASK GENMASK(13, 6)
#define MINDGROVE_SPI_CTRL_TOTAL_BIT_TX(x) ((x) << 6)
#define MINDGROVE_SPI_CTRL_TOTAL_BIT_RX_MASK GENMASK(21, 14)
#define MINDGROVE_SPI_CTRL_TOTAL_BIT_RX(x) ((x) << 14)
#define MINDGROVE_SPI_CTRL_SCLK_OUTEN ((u32)1UL << 22)
#define MINDGROVE_SPI_CTRL_NCS_OUTEN ((u32)1UL << 23)
#define MINDGROVE_SPI_CTRL_MISO_OUTEN ((u32)1UL << 24)
#define MINDGROVE_SPI_CTRL_MOSI_OUTEN ((u32)1UL << 25)

/* CLK_CTRL register bit definitions */
#define MINDGROVE_SPI_CLK_CTRL_POLARITY BIT(0)
#define MINDGROVE_SPI_CLK_CTRL_PHASE BIT(1)
#define MINDGROVE_SPI_CLK_CTRL_PRESCALAR_SHIFT 2
#define MINDGROVE_SPI_CLK_CTRL_PRESCALAR_MASK GENMASK(15, 2)
#define MINDGROVE_SPI_CLK_CTRL_SETUP_SHIFT 16
#define MINDGROVE_SPI_CLK_CTRL_SETUP_MASK GENMASK(23, 16)
#define MINDGROVE_SPI_CLK_CTRL_HOLD_SHIFT 24
#define MINDGROVE_SPI_CLK_CTRL_HOLD_MASK GENMASK(31, 24)

/* FIFO_STATUS register bit definitions */
#define MINDGROVE_SPI_FIFO_STATUS_TX_EMPTY BIT(0)
#define MINDGROVE_SPI_FIFO_STATUS_TX_HALF  BIT(4)
#define MINDGROVE_SPI_FIFO_STATUS_TX_FULL BIT(8)
#define MINDGROVE_SPI_FIFO_STATUS_RX_EMPTY BIT(9)
#define MINDGROVE_SPI_FREQ_STATUS_RX_HALF  BIT(13)
#define MINDGROVE_SPI_FIFO_STATUS_RX_FULL BIT(17)

/* COMM_STATUS register bit definitions */
#define MINDGROVE_SPI_COMM_STATUS_BUSY BIT(0)
#define MINDGROVE_SPI_COMM_STATUS_TX_DEPTH_SHIFT 3
#define MINDGROVE_SPI_COMM_STATUS_TX_DEPTH_MASK GENMASK(5, 3)
#define MINDGROVE_SPI_COMM_STATUS_RX_DEPTH_SHIFT 6
#define MINDGROVE_SPI_COMM_STATUS_RX_DEPTH_MASK GENMASK(8, 6)

/* NCS_CTRL register bit definitions */
#define MINDGROVE_SPI_NCS_CTRL_SELECT(x) ((x) << 0)
#define MINDGROVE_SPI_NCS_CTRL_SW(x) ((x) << 1)

/* Communication modes */
#define MINDGROVE_SPI_COMM_MODE_TX 0
#define MINDGROVE_SPI_COMM_MODE_RX 1
#define MINDGROVE_SPI_COMM_MODE_HALF_DUPLEX 2
#define MINDGROVE_SPI_COMM_MODE_FULL_DUPLEX 3

/*Enablement of different interrupts for SPI*/
#define MINDGROVE_SPI_INTR_TX_FIFO_EMPTY       (1 << 0)
#define MINDGROVE_SPI_INTR_TX_FIFO_HALF        (1 << 4)
#define MINDGROVE_SPI_INTR_TX_FULL             (1 << 8)
#define MINDGROVE_SPI_INTR_RX_EMPTY            (1 << 9)
#define MINDGROVE_SPI_INTR_RX_HALF             (1  << 13)
#define MINDGROVE_SPI_INTR_RX_FULL             (1  << 17)

/*default changes for secure iot*/
#define SECURE_IOT_SPI_WAIT_TX_IDLE     (1U<<0)  //Wait for tx fifo empty.
#define SECURE_IOT_SPI_WAIT_RX_FULL     (1U << 17)  //wait for rx fifo full
#define SECURE_IOT_SPI_WAIT_TX_FULL    (1U<<8)  //Wait for tx fifo empty.
#define SECURE_IOT_SPI_WAIT_RX_EMPTY     (1U << 9)  //wait for rx fifo full
#define SECURE_IOT_SPI_WAIT_TX_HALF       (1U << 4)
#define SECURE_IOT_SPI_WAIT_RX_HALF        (1U << 13)
#define SECURE_IOT_SPI_WAIT_BUSY_CLR    (1U << 2)   //wait for busy =0

struct secure_iot_spi{
    void __iomem *regs; /*virt address of the control registers*/
    struct clk *clk;    /*bus clk set from the dts file*/
    __u8 cs_inactive;
    unsigned int fifo_depth;
    struct completion done;
    struct completion tx_done;
    struct completion rx_done;
    __u32 polling; //1 for polling , 0 for interrupt based.
    struct device *dev;
};

static int secure_iot_spi_init(struct secure_iot_spi *spi){
    /*Interrupts are disabled by default*/
    writel(0,spi->regs+MINDGROVE_SPI_REG_INTR_EN);
    writel(0, spi->regs + MINDGROVE_SPI_REG_CTRL);
    //ctrl |= MINDGROVE_SPI_CTRL_SLAVE_MODE(0); 
    //set by default.

	/* Flush RX FIFO */
	writel(MINDGROVE_SPI_CTRL_RX_FLUSH(1), spi->regs + MINDGROVE_SPI_REG_CTRL);
	writel(MINDGROVE_SPI_CTRL_RX_FLUSH(0), spi->regs + MINDGROVE_SPI_REG_CTRL);
    __u32 regs_out = readl(spi->regs+MINDGROVE_SPI_REG_CTRL);
    regs_out |= (MINDGROVE_SPI_CTRL_SCLK_OUTEN | MINDGROVE_SPI_CTRL_NCS_OUTEN | MINDGROVE_SPI_CTRL_MOSI_OUTEN );
    regs_out &= ~(MINDGROVE_SPI_CTRL_MISO_MODE(1));
    regs_out &= ~(MINDGROVE_SPI_CTRL_SLAVE_MODE(1));
    writel(regs_out, spi->regs + MINDGROVE_SPI_REG_CTRL);


	/* Set default setup and hold times */
	writel((1 << MINDGROVE_SPI_CLK_CTRL_SETUP_SHIFT) |
			   (1 << MINDGROVE_SPI_CLK_CTRL_HOLD_SHIFT),
		   spi->regs + MINDGROVE_SPI_REG_CLK_CTRL);

	/* Configure NCS for software control */
	//writeb(MINDGROVE_SPI_NCS_CTRL_SELECT(1), spi->regs + MINDGROVE_SPI_REG_NCS_CTRL);
    /* Software CS, deasserted (NCS high = idle) */
    writeb(MINDGROVE_SPI_NCS_CTRL_SELECT(1) | MINDGROVE_SPI_NCS_SW(1), spi->regs + MINDGROVE_SPI_REG_NCS_CTRL);
    return 0;
}

static int secure_iot_spi_interrupt_enable(struct secure_iot_spi *spi){
    writel(0xffffffff,spi->regs+MINDGROVE_SPI_REG_INTR_EN);
    return 0;
}
static int secure_iot_spi_prepare_message(struct spi_controller *host, struct spi_message *msg)
{
    struct secure_iot_spi *spi =spi_controller_get_devdata(host);
    struct spi_device *device =msg->spi;
    spi->cs_inactive = spi_get_chipselect(device,0);
    writel(MINDGROVE_SPI_NCS_CTRL_SELECT(spi->cs_inactive)|MINDGROVE_SPI_NCS_CTRL_SELECT(1),spi->regs+MINDGROVE_SPI_REG_NCS_CTRL);
    /* Updating of chip select for initial high. */
    writel( MINDGROVE_SPI_CTRL_COMM_MODE(device->mode)&MINDGROVE_SPI_CTRL_COMM_MODE_MASK,spi->regs+MINDGROVE_SPI_REG_CLK_CTRL);
    /* setting of the clock mode of operation. */
    return 0;
}

static int mindgrove_spi_set_mode(struct secure_iot_spi *spi, u32 mode)
{
	//struct secure_iot_spi *spi = spi_controller_get_devdata(host);
	u32 clk_ctrl;
	u8 ncs_ctrl = 0;

	/* Switch clock mode bits */
	clk_ctrl = readl(spi->regs + MINDGROVE_SPI_REG_CLK_CTRL);
	clk_ctrl &= ~(MINDGROVE_SPI_CLK_CTRL_POLARITY | MINDGROVE_SPI_CLK_CTRL_PHASE);

	if (mode & SPI_CPHA)
		clk_ctrl |= MINDGROVE_SPI_CLK_CTRL_PHASE;
	if (mode & SPI_CPOL)
		clk_ctrl |= MINDGROVE_SPI_CLK_CTRL_POLARITY;

	writel(clk_ctrl, spi->regs + MINDGROVE_SPI_REG_CLK_CTRL);

	/* Set LSB first if required */
	if (mode & SPI_LSB_FIRST)
	{
		u32 ctrl = readl(spi->regs + MINDGROVE_SPI_REG_CTRL) |
				   MINDGROVE_SPI_CTRL_LSBFIRST(1);
		writel(ctrl, spi->regs + MINDGROVE_SPI_REG_CTRL);
	}
	/* Configure NCS control for software mode */
	ncs_ctrl = readl(spi->regs + MINDGROVE_SPI_REG_NCS_CTRL) |
			   MINDGROVE_SPI_NCS_CTRL_SELECT(1);

	/* Update the chip select polarity */
	if (mode & SPI_CS_HIGH)
		ncs_ctrl |= MINDGROVE_SPI_NCS_CTRL_SW(1);

	writeb(ncs_ctrl, spi->regs + MINDGROVE_SPI_REG_NCS_CTRL);
	return 0;
}

static int secure_iot_spi_prep_transfer(struct secure_iot_spi *spi, struct spi_device *device , struct spi_transfer *t)
{
    //for clk control we setup the prescalar.
    __u8 prescalar = DIV_ROUND_UP(clk_get_rate(spi->clk)>>1 ,t->speed_hz -1);
    unsigned int mode;
    prescalar =3;
    __u32 prev_clk = readl(spi->regs+MINDGROVE_SPI_REG_CLK_CTRL);
    prev_clk &= SPI_CLK_PRESCALE(0);
    prev_clk |= SPI_CLK_PRESCALE(prescalar);
    writel(prev_clk,spi->regs+MINDGROVE_SPI_REG_CLK_CTRL);
    //pre scalar configuration made
    /* Mode size setup for fifo block output */
    mode = max_t(unsigned int, t->rx_nbits, t->tx_nbits);
    __u32 ctrl_spi = readl(spi->regs+MINDGROVE_SPI_REG_CTRL);
    ctrl_spi = (MINDGROVE_SPI_CTRL_COMM_MODE(MINDGROVE_SPI_COMM_MODE_FULL_DUPLEX));
    /*Setting the mode of transaction in spi */
    // ctrl_spi &= ~(MINDGROVE_SPI_CTRL_TOTAL_BIT_TX_MASK| MINDGROVE_SPI_CTRL_TOTAL_BIT_TX(0xFF) );   //remove tx configuration.
    // ctrl_spi &= ~(MINDGROVE_SPI_CTRL_TOTAL_BIT_RX_MASK| MINDGROVE_SPI_CTRL_TOTAL_BIT_RX(0xFF) );   //remove rx configuration.
    ctrl_spi &= ~MINDGROVE_SPI_CTRL_TOTAL_BIT_TX_MASK;
    ctrl_spi &= ~MINDGROVE_SPI_CTRL_TOTAL_BIT_RX_MASK;
    ctrl_spi &= ~MINDGROVE_SPI_CTRL_COMM_MODE_MASK;

    switch(mode){
        case 8:
            ctrl_spi |= ((MINDGROVE_SPI_CTRL_TOTAL_BIT_TX_MASK)&MINDGROVE_SPI_CTRL_TOTAL_BIT_TX(8));
            ctrl_spi |= ((MINDGROVE_SPI_CTRL_TOTAL_BIT_RX_MASK)&MINDGROVE_SPI_CTRL_TOTAL_BIT_RX(8));
            break;
        case 16:
            ctrl_spi |= ((MINDGROVE_SPI_CTRL_TOTAL_BIT_TX_MASK)&MINDGROVE_SPI_CTRL_TOTAL_BIT_TX(16));
            ctrl_spi |= ((MINDGROVE_SPI_CTRL_TOTAL_BIT_RX_MASK)&MINDGROVE_SPI_CTRL_TOTAL_BIT_RX(16));
            break;
        case 32:
            ctrl_spi |= ((MINDGROVE_SPI_CTRL_TOTAL_BIT_TX_MASK)&MINDGROVE_SPI_CTRL_TOTAL_BIT_TX(32));
            ctrl_spi |= ((MINDGROVE_SPI_CTRL_TOTAL_BIT_RX_MASK)&MINDGROVE_SPI_CTRL_TOTAL_BIT_RX(32));
            break;  
        default:
            ctrl_spi |= ((MINDGROVE_SPI_CTRL_TOTAL_BIT_TX_MASK)&MINDGROVE_SPI_CTRL_TOTAL_BIT_TX(8));
            ctrl_spi |= ((MINDGROVE_SPI_CTRL_TOTAL_BIT_RX_MASK)&MINDGROVE_SPI_CTRL_TOTAL_BIT_RX(8));
            break;      
    }

    if(device->mode & SPI_LSB_FIRST){
        ctrl_spi |= MINDGROVE_SPI_CTRL_LSBFIRST(1);
    }
    ctrl_spi |= (MINDGROVE_SPI_CTRL_SCLK_OUTEN |
			MINDGROVE_SPI_CTRL_NCS_OUTEN |
			MINDGROVE_SPI_CTRL_MOSI_OUTEN);

	/* Configure control register */
	ctrl_spi |= MINDGROVE_SPI_CTRL_EN(1);
    /*Assuming we finished all the changes to be done for spi*/
    writel(ctrl_spi,spi->regs+MINDGROVE_SPI_REG_CTRL);
    mindgrove_spi_set_mode(spi, device->mode);
    return spi->polling;

}

static irqreturn_t secure_iot_spi_irq(int irq, void *dev_id)
{
    struct secure_iot_spi *spi = dev_id;
    u32 intr_en , fifo_status;
    intr_en = readl(spi->regs + MINDGROVE_SPI_REG_INTR_EN);
    fifo_status = readl(spi->regs + MINDGROVE_SPI_REG_FIFO_STATUS);
    if((intr_en & MINDGROVE_SPI_INTR_TX_FIFO_EMPTY)&& fifo_status & MINDGROVE_SPI_FIFO_STATUS_TX_EMPTY){
        writel(0, spi->regs+MINDGROVE_SPI_REG_INTR_EN);
        complete(&spi->tx_done);
        complete(&spi->done);
        return IRQ_HANDLED;
    }
    if((intr_en & MINDGROVE_SPI_INTR_TX_FIFO_HALF) && (fifo_status & MINDGROVE_SPI_FIFO_STATUS_TX_HALF)){
        writel(0, spi->regs+MINDGROVE_SPI_REG_INTR_EN);
        complete(&spi->tx_done);
        complete(&spi->done);
        /*Tx half fifo interrupt*/
        return IRQ_HANDLED;
    }
    if((intr_en & MINDGROVE_SPI_INTR_TX_FULL) && (fifo_status & MINDGROVE_SPI_FIFO_STATUS_TX_FULL)){
        writel(0, spi->regs+MINDGROVE_SPI_REG_INTR_EN);
        complete(&spi->tx_done);
        complete(&spi->done);
        return IRQ_HANDLED;
    }
    if((intr_en & MINDGROVE_SPI_INTR_RX_EMPTY ) && (fifo_status & MINDGROVE_SPI_FIFO_STATUS_RX_EMPTY)){
        writel(0, spi->regs+MINDGROVE_SPI_REG_INTR_EN);
        complete(&spi->rx_done);
        complete(&spi->done);
        return IRQ_HANDLED;
    }
    if((intr_en & MINDGROVE_SPI_INTR_RX_HALF ) && (fifo_status & MINDGROVE_SPI_FREQ_STATUS_RX_HALF)){
        writel(0, spi->regs+MINDGROVE_SPI_REG_INTR_EN);
        complete(&spi->rx_done);
        complete(&spi->done);
        return IRQ_HANDLED;
    }
    if((intr_en & MINDGROVE_SPI_INTR_RX_FULL ) && (fifo_status & MINDGROVE_SPI_FIFO_STATUS_RX_FULL)){
        writel(0, spi->regs+MINDGROVE_SPI_REG_INTR_EN);
        complete(&spi->rx_done);
        complete(&spi->done);
        return IRQ_HANDLED;
    }
    writel(0, spi->regs+MINDGROVE_SPI_REG_INTR_EN);
    
    return IRQ_NONE;
}

static void secure_iot_spi_set_cs(struct spi_device *device, bool is_high)
{
    struct secure_iot_spi *spi = spi_controller_get_devdata(device->controller);
    if(is_high){
        writel(MINDGROVE_SPI_NCS_CTRL_SW(1)|MINDGROVE_SPI_NCS_CTRL_SELECT(1), spi->regs+MINDGROVE_SPI_REG_NCS_CTRL);
    }
    //setting the chip select value to low to turn on.
    else{
        writel(MINDGROVE_SPI_NCS_CTRL_SW(0)|MINDGROVE_SPI_NCS_CTRL_SELECT(1), spi->regs+MINDGROVE_SPI_REG_NCS_CTRL);
    }
}

// spi wait for completion logic : 
static void secure_iot_spi_wait (struct secure_iot_spi *spi, u32 bit, int poll){
    unsigned long timeout = 1000000;
    int use_polling = poll;
    if(use_polling){
        __u32 fifo_status;
        __u16 comm_status;

        /*polling loop*/
        /*Polling mode for faster transfers */
        while(timeout -- >0){
            fifo_status = readl(spi->regs + MINDGROVE_SPI_REG_FIFO_STATUS);
            comm_status = readw(spi->regs +MINDGROVE_SPI_REG_COMM_STATUS);
            bool tx_idle = (fifo_status & MINDGROVE_SPI_FIFO_STATUS_TX_EMPTY);
            bool rx_full = (fifo_status & MINDGROVE_SPI_FIFO_STATUS_RX_FULL);
            //bool busy = (fifo_status & SECURE_IOT_SPI_WAIT_BUSY_CLR);
            bool busy    = (comm_status & MINDGROVE_SPI_COMM_STATUS_BUSY);
            //Check functioning 
            
            //Check conditions based on bit mask :
            if((bit & SECURE_IOT_SPI_WAIT_TX_IDLE) && tx_idle){
                if( !(bit & SECURE_IOT_SPI_WAIT_BUSY_CLR) || !busy){
                    return;
                }
            }

            if((bit & SECURE_IOT_SPI_WAIT_RX_FULL)&& rx_full){
                return;
            }

            udelay(1);
        }
    }
    else{
        /*interrupt based : enable interrupt and wait.*/
        u32 intr_en = readl(spi->regs+MINDGROVE_SPI_REG_INTR_EN);
        if(bit & SECURE_IOT_SPI_WAIT_TX_FULL){
            intr_en |= MINDGROVE_SPI_INTR_TX_FULL;
        }

        if(bit & SECURE_IOT_SPI_WAIT_RX_EMPTY){
            intr_en |= MINDGROVE_SPI_INTR_RX_EMPTY;
        }

        /*Save the state if needed */
        reinit_completion(&spi->done); 
        //To see if the transaction is completed via done
        writel(intr_en, spi->regs+MINDGROVE_SPI_REG_INTR_EN);
        wait_for_completion(&spi->done);

        /*optionally clear interrupt*/
        //writel(0, spi->regs+MINDGROVE_SPI_REG_INTR_EN);
    }
}

static int secure_iot_spi_wait_rx_data(struct secure_iot_spi *spi)
{
    u32 status;
    return readl_poll_timeout_atomic(spi->regs + MINDGROVE_SPI_REG_FIFO_STATUS, 
                                     status, !(status & MINDGROVE_SPI_FIFO_STATUS_RX_EMPTY), 
                                     1, 100000);
}

static int secure_iot_spi_wait_tx_data(struct secure_iot_spi *spi)
{
    u32 status;
    
    /* * Read FIFO_STATUS until TX_EMPTY (Bit 0) is 1.
     * Check every 1us, timeout after 100ms (100,000us).
     */
    return readl_poll_timeout_atomic(spi->regs + MINDGROVE_SPI_REG_FIFO_STATUS, 
                                     status, !(status & MINDGROVE_SPI_FIFO_STATUS_TX_FULL), 
                                     1, 100000);
}

static void secure_iot_spi_tx(struct secure_iot_spi *spi, const u8 *tx_ptr)
{
    WARN_ON_ONCE((readl(spi->regs+MINDGROVE_SPI_REG_FIFO_STATUS)& MINDGROVE_SPI_INTR_TX_FULL) != 0);
    //If tx is already full we cant write in new data for filling it in right.
    writel(*tx_ptr, spi->regs+MINDGROVE_SPI_REG_TX );
}

static void secure_iot_spi_rx(struct secure_iot_spi *spi, u8 *rx_ptr)
{
    u32 data = readl(spi->regs+MINDGROVE_SPI_REG_RX);
    WARN_ON_ONCE((readl(spi->regs+MINDGROVE_SPI_REG_FIFO_STATUS)&MINDGROVE_SPI_INTR_RX_EMPTY)!=0);
    //If rx is already empty we cant read new data from the rx buffer.
    *rx_ptr = data;
    /*no nedd to mask it as its the entire data segment.*/
}

static int secure_iot_spi_transfer_one(struct spi_controller *host, struct spi_device *device , struct spi_transfer *t)
{
    //getting the remaining objects from the spi transfer object.
    struct secure_iot_spi *spi = spi_controller_get_devdata(host);
    int poll = secure_iot_spi_prep_transfer(spi,device ,t);
    // as a transceive function operation:
    const u8 *tx_ptr = t->tx_buf;
    u8* rx_ptr = t->rx_buf;
    unsigned int remaining_words = t->len;

    while(remaining_words){
        // unsigned int n_words = min(remaining_words, spi->fifo_depth);
        // unsigned int i;

        // /* enqueue n words for transfer of data */
        // for(int i=0;i<n_words;i++){
        //     secure_iot_tx(spi, tx_ptr++);
        // }
        /*rx logic with delay passing */

        //sending of data using delay in between.
        //calling of the wait function.
        //secure_iot_spi_wait(spi, SECURE_IOT_SPI_WAIT_TX_IDLE, spi->polling);
        //secure_iot_spi_wait_done(spi);
        //Wait for the tx buffer to be empty after the sending of the data 
        secure_iot_spi_interrupt_enable(spi);
        int ret = secure_iot_spi_wait_tx_data(spi);
        if (ret) {
                dev_err(spi->dev, "TX timeout\n");
                return ret;
        }
        secure_iot_spi_tx(spi,tx_ptr++);
        if(rx_ptr){
            //secure_iot_spi_wait(spi,SECURE_IOT_SPI_WAIT_RX_FULL,spi->polling);
            //secure_iot_spi_wait_done(spi); --> should find either tx/rx
            //wait for any data to be present in rx for access.
            secure_iot_spi_interrupt_enable(spi);
            ret = secure_iot_spi_wait_rx_data(spi);
            if (ret) {
                dev_err(spi->dev, "RX timeout\n");
                return ret;
            }
            secure_iot_spi_rx(spi,rx_ptr++);
        }
        remaining_words--;
    }
    spi_finalize_current_transfer(host);
    return 0;
}

static void secure_iot_spi_remove(struct platform_device *pdev)
{
    struct spi_controller *host = platform_get_drvdata(pdev);
    struct secure_iot_spi *spi = spi_controller_get_devdata(host);
    /*disable all the interrupts*/
    writel(0,spi->regs+MINDGROVE_SPI_REG_INTR_EN);
    clk_disable_unprepare(spi->clk);
}

static int secure_iot_spi_probe(struct platform_device *pdev){
    struct secure_iot_spi *spi;
    int ret, irq, num_cs;
    u32 cs_bits ,max_bits_per_word;
    struct spi_controller *host;

    host= spi_alloc_host(&pdev->dev, sizeof(struct secure_iot_spi));
    // Instead of host = spi_alloc_host(...)
    //host = spi_alloc_master(&pdev->dev, sizeof(struct secure_iot_spi)); 
    if(!host){
        dev_err(&pdev->dev ," Out of memory \n");
        return -ENOMEM;
    }
    spi =spi_controller_get_devdata(host);

    init_completion(&spi->done);
    init_completion(&spi->tx_done);   // ADD THIS
    init_completion(&spi->rx_done);   // ADD THIS

    spi->dev = &pdev->dev;
    init_completion(&spi->done);
    platform_set_drvdata(pdev,host);

    spi->regs = devm_platform_ioremap_resource(pdev,0);
    if(IS_ERR(spi->regs)){
        ret = PTR_ERR(spi->regs);
        goto put_host;
    }

    spi->clk = devm_clk_get(&pdev->dev,NULL);
    if(IS_ERR(spi->clk)) {
        dev_err(&pdev->dev, "Unable to find bus clock \n");
        ret = PTR_ERR(spi->clk);
        goto put_host;
    }

    irq = platform_get_irq(pdev,0);
    // Is it generic plic or very specific to secure_iot hardware?
    if(irq < 0){
        ret = irq;
        goto put_host;
    }

    /*optional paramenters input to spi from dts input*/
    ret = of_property_read_u32(pdev->dev.of_node,"mindgrove,fifo-depth",
                                                    &spi->fifo_depth);
    if(ret < 0){
        spi->fifo_depth = MINDGROVE_SPI_DEFAULT_DEPTH;
    }
    ret = of_property_read_u32(pdev->dev.of_node,"mindgrove,max-bits-per-word",
                                                    &max_bits_per_word);
    if(!ret && max_bits_per_word < 8){
        dev_err(&pdev->dev ,"Only 8bit SPI words supported by the driver \n");
        ret = -EINVAL;
        goto put_host;
    }
    /* Spin up the clock before hitting on the registers. */
    ret = clk_prepare_enable(spi->clk);
    if(ret) {
        dev_err(&pdev->dev," Unable to enable the clock bus in the system bus\n");
        goto put_host;
    }

    if (of_property_read_u32(pdev->dev.of_node, "num-cs", &num_cs))
        num_cs = 1; // Default to 1 if not in DTS


    /*defining of the host for argument passing*/
    //host->bus_num = pdev->id;
    // host->bus_num = of_alias_get_id(pdev->dev.of_node, "spi");
    // if (host->bus_num < 0)
    //     host->bus_num = -1;  /* let kernel auto-assign if no alias */
    u32 bus_num;
    if (of_property_read_u32(pdev->dev.of_node, "mindgrove,bus-num", &bus_num))
        host->bus_num = -1;  /* auto-assign */
    else
        host->bus_num = bus_num;
    host->num_chipselect = num_cs;
    //host->mode_bits = (SPI_CPHA&1) | (SPI_CPOL&(1<<1)) | (SPI_LSB_FIRST&(1<<3) ) | (SPI_CS_HIGH & (1<<2));
    host->mode_bits = SPI_CPHA | SPI_CPOL | SPI_LSB_FIRST | SPI_CS_HIGH;
    /*Check what is the case of this implementation. */
    host->bits_per_word_mask = SPI_BPW_MASK(8);
    host->prepare_message = secure_iot_spi_prepare_message;
    host->set_cs = secure_iot_spi_set_cs;
    host->transfer_one = secure_iot_spi_transfer_one;
    pdev->dev.dma_mask =NULL;
    spi->polling=1; 
    /*Initial flag check for fully using polling for transfer.*/
    /*configure spi host hardware*/

    secure_iot_spi_init(spi);
    /*register for spi interrupt*/
    ret = devm_request_irq(&pdev->dev , irq , secure_iot_spi_irq, 0, dev_name(&pdev->dev),spi);
    if(ret){
        dev_err(&pdev->dev ,"Unable to find the interrupt\n");
        goto disable_clk;
    }
    //debug info for getting information:
    dev_info(&pdev->dev , "mapped: irq=%d , cs =%d\n",irq,host->num_chipselect);
    ret = devm_spi_register_controller(&pdev->dev,host);
    if(ret < 0){
        dev_err(&pdev->dev ,"Spi register host failed to happen !\n");
        goto disable_clk;
    }

    return 0;

    disable_clk:
        clk_disable_unprepare(spi->clk);
    put_host:
        spi_controller_put(host);

        return ret;

}

static int secure_iot_spi_suspend(struct device *dev)
{
    struct spi_controller *host =dev_get_drvdata(dev);
    struct secure_iot_spi *spi = spi_controller_get_devdata(host);
    int ret;
    ret = spi_controller_suspend(host);
    if(ret){
        return ret;
    }
    /*disabling of all the interrupts again just in case*/
    writel(0,spi->regs+MINDGROVE_SPI_REG_INTR_EN);
    clk_disable_unprepare(spi->clk);
    return ret;
}

static int secure_iot_spi_resume(struct device *dev)
{
    struct spi_controller *host = dev_get_drvdata(dev);
    struct secure_iot_spi *spi = spi_controller_get_devdata(host);
    int ret;
    ret = clk_prepare_enable(spi->clk);
    if(ret){
        return ret;
    }
    ret = spi_controller_resume(host);
    if(ret){
        clk_disable_unprepare(spi->clk);
    }
    return ret;

}



/********************************************************
 * Basic struct definitions for spi recognitons
 * 
 * Call backs links defined
 * 
 *********************************************************/

static DEFINE_SIMPLE_DEV_PM_OPS(secure_iot_spi_pm_ops,
				secure_iot_spi_suspend, secure_iot_spi_resume);


static const struct of_device_id mindgrove_spi_of_match[] = {
	{ .compatible = "mindgrove,spi", },
	{}
};
MODULE_DEVICE_TABLE(of, mindgrove_spi_of_match);

static struct platform_driver mindgrove_spi_driver = {
	.probe = secure_iot_spi_probe,
	.remove = secure_iot_spi_remove,
	.driver = {
		.name = SECURE_IOT_SPI_DRIVER_NAME,
		.pm = &secure_iot_spi_pm_ops,
		.of_match_table = mindgrove_spi_of_match,
	},
};

module_platform_driver(mindgrove_spi_driver);

MODULE_AUTHOR("Biancaa Ramesh <biancaa2210329@ssn.edu.in>");
MODULE_DESCRIPTION("SecureIoT SPI driver");
MODULE_LICENSE("GPL");