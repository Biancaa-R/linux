#include<linux/kernel.h>
#include<linux/module.h>
#include<linux/moduleparam.h>
#include<linux/types.h>
#include<linux/bitfield.h>
#include<linux/clk.h>
#include<linux/io.h>
#include<linux/math.h>
#include<linux/of.h>
#include<linux/platform_device.h>
//for defining the peripherals as platform device  and assign memory.
#include <linux/watchdog.h>

typedef struct {                                /*!< WDT Structure                                                             */
  __IOM uint32_t  WDT_CYCLES;                   /*!< The number of cycles to count down for reset generation                   */
  __IM  uint32_t  RESERVED;
  
  union {
    __IOM uint16_t WDT_CTRL;                    /*!< Control register                                                          */
    
    struct {
      __IOM uint16_t WDT_CTRL_EN : 1;           /*!< Enable the watchdog timer                                                 */
      __IOM uint16_t WDT_CTRL_MODE : 1;         /*!< Mode of watchdog timer. 0 - Interrupt, 1 - Reset                          */
      __IOM uint16_t WDT_CTRL_SOFT : 1;         /*!< Software reset                                                            */
            uint16_t            : 13;
    } WDT_CTRL_b;
  } ;
  __IM  uint16_t  RESERVED1;
  __IM  uint32_t  RESERVED2;
  __IOM uint16_t  WDT_RESET_CYCLES;             /*!< The number of cycles for which the interrupt needs to be held             */
  __IM  uint16_t  RESERVED3;
  __IM  uint32_t  RESERVED4;
  __IOM uint32_t  WDT_ACTIVE;                   /*!< Update the internal WD counter with the WD_CYCLES register                */
} WDT_Type;                                     /*!< Size = 28 (0x1c)                                                          */


struct secure_iot_wdt_desc{
    struct watchdog_device wdog_dev;
    unsigned int wdt_freq;
    void __iomem *base;
    WDT_Type* wdt;
}

//Module parameter of the heartbeat interval 
//HeartbeatInterval<WatchdogTimeout

#define WDT_HEARTBEAT 5000
static int heartbeat WDT_HEARTBEAT;
//heartbeat =5000;
// WE dont have provision for modifying the heart beat of the wdt
#define WDT_CYCLES 5000
static int wdt_cycles= WDT_CYCLES;

// module_param(heartbeat,int ,0);
// MODULE_PARM_DESC(heartbeat, "Watchdog heartbeats in seconds. (default="
// 		 __MODULE_STRING(WDT_HEARTBEAT) ")");

module_param(wdt_cycles,int,0);
MODULE_PARAM_DESC(wdt_cycles,"Watchdog cycles count in seconds. (default ="__MODULE_STRING(WDT_CYCLES)")");

static void secure_iot_wdt_init(struct platform_device *pdev){
    dev_dbg(&pdev->dev,"Watch dog timer detected successfully in the system");
}

static void secure_iot_wdt_remove(struct platform_device *pdev){
    dev_dbg(&pdev->dev,"Watch dog timer removed successfully");
}

static int secure_iot_wdt_start(struct watchdog_device *wdog_dev){
    struct secure_iot_wdt_desc *seciot_wdt=watchdog_get_drvdata(wdog_dev);
    seciot_wdt->wdt->WDT_CTRL_b.WDT_CTRL_EN=1;
    seciot_wdt->wdt->WDT_CTRL_b.WDT_CTRL_MODE=1; //reset mode of the watch dog timer
    // I think we dont need sw reset at this point
    seciot_wdt->wdt->WDT_CTRL_b.WDT_CTRL_SOFT=0;
    secure_iot->wdt->WDT_ACTIVE=1;
    u32 cycles;

    cycles = wdog_dev->timeout * seciot_wdt->wdt_freq;

    seciot_wdt->wdt->WDT_CYCLES = cycles;
    seciot_wdt->wdt->WDT_CYCLES= wdt_cycles;
    //for now we are getting the cycles as input.
    //directly passing the module parameter or it needs to be stored first.
    return 0;
}

static int secure_iot_wdt_stop(struct watchdog_device *wdog_dev){
    //stop is basically disabling the watch dog timers
    struct secure_iot_wdt_desc *seciot_wdt=watchdog_get_drvdata(wdog_dev);
    //seciot_wdt->wdt->WDT_CTRL_b &= 0xFFFE;
    //seciot_wdt->wdt->WDT_CTRL_b.WDT_CTRL_EN = 0;
    //if we want to access the entire register fullty
    seciot_wdt->wdt->WDT_CTRL &= ~BIT(0);

}

//the reloading of the watch dog timer is done as wdt ping function.
static int secure_iot_wdt_ping(struct watchdog_device *wdog_dev){
    struct secure_iot_wdt_desc *seciot_wdt =watchdog_get_drvdata(wdog_dev);
    secure_iot_wdt_stop(wdog_dev);
    secure_iot_wdt_start(wdog_dev);
    return 0;

}

static int watchdog_active(struct watchdog_device *wdog_dev){
    struct secure_iot_wdt_desc *seciot_wdt=watchdog_get_drvdata(wdog_dev);
    if (seciot_wdt->wdt->WDT_ACTIVE & 0x1 ==1){
        //it is set so active
        return 1;
    }
    return 0;
}
static int secure_iot_wdt_set_timeout(struct watchdog_device *wdog_dev,uint32_t timeout){
    struct secure_iot_wdt_desc *seciot_wdt=watchdog_get_drvdata(wdog_dev);
    wdog_dev->timeout=timeout;
    if(watchdog_active(wdog_dev)){
        secure_iot_wdt_stop(wdog_dev);
    }
    wdt_cycles=timeout* (uint32_t)(sec_iot->wdt_freq);
    seciot_wdt->wdt->WDT_CYCLES=wdt_cycles;
    return secure_iot_wdt_start(wdog_dev);
    //return 0;
}

static const struct watchdog_info secure_iot_wdt_info={
    .options = WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE | WDIOF_KEEPALIVEPING,
    .identity ="SecureIoT watchdog",
};
static int secure_iot_wdt_probe(struct platform_device *pdev){
    secure_iot_wdt_init(pdev);
    struct secure_iot_wdt_desc *seciot_wdt;
    struct watchdog_device *wdog_dev;
    struct device *dev =&pdev->dev;
    struct clk* bus_clk;
    int ret;

    seciot_wdt =devm_kzalloc(dev,sizeof(*seciot_wdt),GFP_KERNEL);
    if (!seciot_wdt){
        return -ENOMEM;
    }
    seciot_wdt->base =devm_platform_ioremap_resource(pdev,0);
    if(IS_ERR(seciot_wdt->base)){
        return PTR_ERR(seciot_wdt->base);
    }
    seciot_wdt->wdt =(WDT_Type*)seciot_wdt->base;
    //making the watch dog timer stsruct on top of the virtual address.
    bus_clk =devm_clk_get_enabled(dev,"bus");
    if(IS_ERR(bus_clk)){
        return dev_err_probe(dev,PTR_ERR(bus_clk),"failed to enable bus clock\n");
    }

    //wdt clicks at half of the bus rate
    seciot_wdt->wdt_freq= clk_get_rate(bus_clk)/2;
    wdog_dev =&seciot_wdt->wdog_dev;
    //getting input as timeout
    wdog_dev->timeout=heartbeat;
    wdog_dev->info=&secure_iot_wdt_info;
    wdog_dev->ops=&secure_iot_wdt_ops;
    //wdog_dev->max_timeout=FIELD_MAX(WDT_TIMER_VAL)/seciot_wdt->wdt_freq;
    //It is not defined in our case of implementation.
    wdog_dev->max_timeout =U32_MAX / seciot_wdt->wdt_freq;

    wdog_dev->parent=dev;
    //common to all the wdt drivers added
    watchdog_set_drvdata(wdog_dev,seciot_wdt);
    watchdog_stop_on_unregister(wdog_dev);

    ret = devm_watchdog_register_device(dev,wdog_dev);
    if(ret){
        return ret;
    }
    platform_set_drvdata(pdev,seciot_wdt);
    return 0;
}

static int secure_iot_wdt_suspend(struct device*dev){
    struct secure_iot_wdt_desc *seciot_wdt =dev_get_drvdata(dev);
    if(watchdog_active(&seciot_wdt->wdog_dev)){
        secure_iot_wdt_stop(&seciot_wdt->wdog_dev);
    }
    return 0;
}

static const struct watchdog_ops secure_iot_wdt_ops ={
    .owner=THIS_MODULE,
    .start=secure_iot_wdt_start,
    .stop=secure_iot_wdt_stop,
    .ping=secure_iot_wdt_ping,
    .set_timeout=secure_iot_wdt_set_timeout,
    //.get_timeleft=secure_iot_wdt_get_timeleft,
    //no provision for it in hw
};

static const struct of_device_id seciot_wdt_of_match[] ={
    { .compatible="mindgrove,secure_iot-wdt"},
    { },
};

MODULE_DEVICE_TABLE(of,seciot_wdt_of_match);

static struct platform_driver secure_iot_wdt_driver={
    .probe=secure_iot_wdt_probe,
    .driver={
        .name="secure_iot-wdt",
        .of_match_table=seciot_wdt_of_match,
    },
};

module_platform_driver(secure_iot_wdt_driver);
MODULE_AUTHOR("Biancaa Ramesh <biancaa2210329@ssn.edu.in>");
MODULE_DESCRIPTION("Secure_iot WDT drivers\n");
MODULE_LICENSE("GPL");