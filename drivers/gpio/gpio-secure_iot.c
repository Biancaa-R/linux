// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Mindgrove Technologies Pvt. Ltd.
 * Author : Biancaa Ramesh <biancaa2210329@ssn.edu.in>
 * Driver for Mindgrove Silicon's GPIO Peripheral
 *
 * Documentation: Mindgrove Secure IoT SoC GPIO Peripheral Reference Manual
 *
 * This driver supports up to 45 GPIO pins, split between two registers (GPIO_REG and GPIO_PINMUX_REG).
 * It provides direction control, pin read/write, and interrupt configuration for each pin.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/gpio/driver.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/internal.h>

//#include "gpio.h"
//#include "pinmux.h"
//#include "log.h"
//#include "errors.h"

//#define PINMUX0_BASE                0x00040400UL


typedef struct {                                /*!< PINMUX0 Structure                                                         */
  __IOM uint32_t  MUX0;                         /*!< Select between GPIO0 and PWM0. 0 - GPIO, 1 - PWM                          */
  __IOM uint32_t  MUX1;                         /*!< Select between GPIO1 and PWM1. 0 - GPIO, 1 - PWM                          */
  __IOM uint32_t  MUX2;                         /*!< Select between GPIO2 and PWM2. 0 - GPIO, 1 - PWM                          */
  __IOM uint32_t  MUX3;                         /*!< Select between GPIO3 and PWM3. 0 - GPIO, 1 - PWM                          */
  __IOM uint32_t  MUX4;                         /*!< Select between GPIO4 and PWM4. 0 - GPIO, 1 - PWM                          */
  __IOM uint32_t  MUX5;                         /*!< Select between GPIO5 and PWM5. 0 - GPIO, 1 - PWM                          */
  __IOM uint32_t  MUX6;                         /*!< Select between GPIO6 and PWM6. 0 - GPIO, 1 - PWM                          */
  __IOM uint32_t  MUX7;                         /*!< Select between GPIO7 and PWM7. 0 - GPIO, 1 - PWM                          */
  __IOM uint32_t  MUX8;                         /*!< Select between GPIO17 and PWM8. 0 - GPIO, 1 - PWM                         */
  __IOM uint32_t  MUX9;                         /*!< Select between GPIO18 and PWM9. 0 - GPIO, 1 - PWM                         */
  __IOM uint32_t  MUX10;                        /*!< Select between GPIO19 and PWM10. 0 - GPIO, 1 - PWM                        */
  __IOM uint32_t  MUX11;                        /*!< Select between GPIO20 and PWM11. 0 - GPIO, 1 - PWM                        */
  __IOM uint32_t  MUX12;                        /*!< Select between GPIO21 and PWM12. 0 - GPIO, 1 - PWM                        */
  __IOM uint32_t  MUX13;                        /*!< Select between GPIO22 and PWM13. 0 - GPIO, 1 - PWM                        */
  __IOM uint32_t  MUX14;                        /*!< Select between GPIOP0 and SPI2 MOSI. 1 - GPIO, 0 - SPI2 MOSI              */
  __IOM uint32_t  MUX15;                        /*!< Select between GPIOP1 and SPI2 MISO. 1 - GPIO, 0 - SPI2 MISO              */
  __IOM uint32_t  MUX16;                        /*!< Select between GPIOP2 and SPI2 SCLK. 1 - GPIO, 0 - SPI2 SCLK              */
  __IOM uint32_t  MUX17;                        /*!< Select between GPIOP4 and SPI2 NCS. 1 - GPIO, 0 - SPI2 NCS                */
  __IOM uint32_t  MUX18;                        /*!< Select between GPIOP5 and SPI3 MOSI. 1 - GPIO, 0 - SPI3 MOSI              */
  __IOM uint32_t  MUX19;                        /*!< Select between GPIOP6 and SPI3 MISO. 1 - GPIO, 0 - SPI3 MISO              */
  __IOM uint32_t  MUX20;                        /*!< Select between GPIOP7 and SPI3 SCLK. 1 - GPIO, 0 - SPI3 SCLK              */
  __IOM uint32_t  MUX21;                        /*!< Select between GPIOP8 and SPI3 NCS. 1 - GPIO, 0 - SPI3 NCS                */
  __IOM uint32_t  MUX22;                        /*!< Select between GPIOP9 and JTAG TDO. 1 - GPIO, 0 - JTAG TDO                */
  __IOM uint32_t  MUX23;                        /*!< Select between GPIOP10 and JTAG TDI. 1 - GPIO, 0 - JTAG TDI               */
  __IOM uint32_t  MUX24;                        /*!< Select between GPIOP11 and JTAG TMS. 1 - GPIO, 0 - JTAG TMS               */
  __IOM uint32_t  MUX25;                        /*!< Select between GPIOP12 and JTAG TCLK. 1 - GPIO, 0 - JTAG TCLK             */
  __IOM uint32_t  MUX26;                        /*!< Select between GPIOP13 and JTAG TRST. 1 - GPIO, 0 - JTAG TRST             */
  __IOM uint32_t  MUX27;                        /*!< Select between GPIO8 and UART3 TX. 1 - GPIO, 0 - UART3 TX                 */
  __IOM uint32_t  MUX28;                        /*!< Select between GPIO9 and UART3 RX. 1 - GPIO, 0 - UART3 RX                 */
  __IOM uint32_t  MUX29;                        /*!< Select between GPIO11 and UART4 TX. 1 - GPIO, 0 - UART4 TX                */
  __IOM uint32_t  MUX30;                        /*!< Select between GPIO15 and UART4 TX. 1 - GPIO, 0 - UART4 RX                */
} PINMUX_Type;                                 /*!< Size = 124 (0x7c)                                                         */


/**
  * @brief General purpose IO. 32 GPIOs are available (GPIO)
  */

typedef struct {                                /*!< GPIO Structure                                                            */
  __IOM uint32_t  GPIO_DIRECTION;               /*!< Select the direction of the GPIOs. Each bit position corresponds
                                                     to the respective GPIO pin. 0 - Output, 1 - Input                         */
  __IM  uint32_t  RESERVED;
  __IOM uint32_t  GPIO_DATA;                    /*!< Contains the data to be sent out if the GPIO pin is configured
                                                     as output and the data recieved when configured as input.
                                                     Each bit position corresponds to the respective GPIO pin.                 */
  __IM  uint32_t  RESERVED1;
  __IOM uint32_t  GPIO_SET;                     /*!< To set the respective GPIO pins                                           */
  __IM  uint32_t  RESERVED2;
  __IOM uint32_t  GPIO_CLEAR;                   /*!< To clear the respective GPIO pins                                         */
  __IM  uint32_t  RESERVED3;
  __IOM uint32_t  GPIO_TOGGLE;                  /*!< To invert the respective GPIO pins                                        */
  __IM  uint32_t  RESERVED4[3];
  __IOM uint32_t  GPIO_INTR;                    /*!< To enable the interrupt of respective GPIO pins                           */
  __IM  uint32_t  RESERVED5;
  __IOM uint32_t  GPIO_PULLUP_CONFIG;           /*!< To enable the interrupt of respective GPIO pins                           */
  __IM  uint32_t  RESERVED6;
  
  union {
    __IOM uint32_t GPIO_BUFFER_CONTROL;         /*!< To control and change the gpio buffer parameters                          */
    
    struct {
      __IOM uint32_t Buffer_2_Enable : 1;       /*!< 1 : 2 Bit buffer enabled ; 0 : 2 Bit buffer disabled                      */
      __IOM uint32_t Buffer_4_Enable : 1;       /*!< 1 : 4 Bit buffer enabled ; 0 : 4 Bit buffer disabled                      */
      __IOM uint32_t Buffer_8_Enable : 1;       /*!< 1 : 8 Bit buffer enabled ; 0 : 8 Bit buffer disabled                      */
      __IOM uint32_t Buffer_2_clock_Select : 1; /*!< 0 : Buffer 2 operates on internal clock ; 1 : Buffer 2 operates
                                                     on external clock received through gpio pin 5                             */
      __IOM uint32_t Buffer_4_clock_Select : 1; /*!< 0 : Buffer 4 operates on internal clock ; 1 : Buffer 4 operates
                                                     on external clock received through gpio pin 6                             */
      __IOM uint32_t Buffer_8_clock_Select : 1; /*!< 0 : Buffer 8 operates on internal clock ; 1 : Buffer 8 operates
                                                     on external clock received through gpio pin 7                             */
      __IOM uint32_t Buffer_2_Clock_Edge_Select : 1;/*!< 0 : Buffer 2 enqueues and dequeues on positive edge of selected
                                                     clock ; 1 : Buffer 2 enqueues and dequeues on negative
                                                     edge of selected clock                                                    */
      __IOM uint32_t Buffer_4_Clock_Edge_Select : 1;/*!< 0 : Buffer 4 enqueues and dequeues on positive edge of selected
                                                     clock ; 1 : Buffer 4 enqueues and dequeues on negative
                                                     edge of selected clock                                                    */
      __IOM uint32_t Buffer_8_Clock_Edge_Select : 1;/*!< 0 : Buffer 8 enqueues and dequeues on positive edge of selected
                                                     clock ; 1 : Buffer 8 enqueues and dequeues on negative
                                                     edge of selected clock                                                    */
      __IOM uint32_t Buffer_2_direction : 1;    /*!< When 0 : buffer 2 enqueue from GPIO is enabled ; 1 : buffer
                                                     2 dequeue to GPIO enabled                                                 */
      __IOM uint32_t Buffer_4_direction : 1;    /*!< When 0 : buffer 4 enqueue from GPIO is enabled ; 1 : buffer
                                                     4 dequeue to GPIO enabled                                                 */
      __IOM uint32_t Buffer_8_direction : 1;    /*!< When 0 : buffer 8 enqueue from GPIO is enabled ; 1 : buffer
                                                     8 dequeue to GPIO enabled                                                 */
      __IOM uint32_t Buffer_2_clear : 1;        /*!< clears buffer 2 data and makes the buffer empty                           */
      __IOM uint32_t Buffer_4_clear : 1;        /*!< clears buffer 4 data and makes the buffer empty                           */
      __IOM uint32_t Buffer_8_clear : 1;        /*!< clears buffer 8 data and makes the buffer empty                           */
      __IOM uint32_t Buffer_2_data_check : 1;   /*!< Size for data availability check in buffer 2 ; 00 : not checking
                                                     for any data ; 01 : checking for 8 bits send or receive
                                                     ; 10 : checking for 16 bits send or receive ; 11 : checking
                                                     for 32 bits send or receive                                               */
      __IOM uint32_t Buffer_4_data_check : 1;   /*!< Size for data availability check in buffer 4 ; 00 : not checking
                                                     for any data ; 01 : checking for 8 bits send or receive
                                                     ; 10 : checking for 16 bits send or receive ; 11 : checking
                                                     for 32 bits send or receive                                               */
      __IOM uint32_t Buffer_8_data_check : 1;   /*!< Size for data availability check in buffer 8 ; 00 : not checking
                                                     for any data ; 01 : checking for 8 bits send or receive
                                                     ; 10 : checking for 16 bits send or receive ; 11 : checking
                                                     for 32 bits send or receive                                               */
      __IOM uint32_t Buffer_12_data_check : 1;  /*!< Size for data availability check in buffer 4 and 8 combined
                                                     as one ; 00 : not checking for any data ; 01 : checking
                                                     for 8 bits send or receive ; 10 : checking for 16 bits
                                                     send or receive ; 11 : checking for 32 bits send or receive               */
            uint32_t            : 13;
    } GPIO_BUFFER_CONTROL_b;
  } ;
  __IM  uint32_t  RESERVED7;
  
  union {
    __IOM uint32_t GPIO_BUFFER_STATUS;          /*!< To read the status of the buffers                                         */
    
    struct {
      __IOM uint32_t Buffer_2_Not_Full : 1;     /*!< Stores 1 if there is space in buffer 2 for an enqueue ; 0 if
                                                     buffer 2 is full                                                          */
      __IOM uint32_t Buffer_2_Not_Empty : 1;    /*!< stores 1 if there is an element to dequeue in buffer 2 ; 0 if
                                                     buffer 2 is empty                                                         */
      __IOM uint32_t Buffer_4_Not_Full : 1;     /*!< Stores 1 if there is space in buffer 4 for an enqueue ; 0 if
                                                     buffer 4 is full                                                          */
      __IOM uint32_t Buffer_4_Not_Empty : 1;    /*!< stores 1 if there is an element to dequeue in buffer 4 ; 0 if
                                                     buffer 4 is empty                                                         */
      __IOM uint32_t Buffer_8_Not_Full : 1;     /*!< Stores 1 if there is space in buffer 8 for an enqueue ; 0 if
                                                     buffer 8 is full                                                          */
      __IOM uint32_t Buffer_8_Not_Empty : 1;    /*!< Stores 1 if there is an element to dequeue in buffer 8 ; 0 if
                                                     buffer 8 is empty                                                         */
      __IOM uint32_t Buffer_2_can_take_input_for_dma : 1;/*!< Stores 1 if there is space available in buffer 2 for the size
                                                     requested by dma, else stores 0                                           */
      __IOM uint32_t Buffer_2_output_ready_for_dma : 1;/*!< Stores 1 if there is data available in buffer 2 for the size
                                                     requested by dma, else stores 0                                           */
      __IOM uint32_t Buffer_4_can_take_input_for_dma : 1;/*!< Stores 1 if there is space available in buffer 4 for the size
                                                     requested by dma, else stores 0                                           */
      __IOM uint32_t Buffer_4_output_ready_for_dma : 1;/*!< Stores 1 if there is data available in buffer 4 for the size
                                                     requested by dma, else stores 0                                           */
      __IOM uint32_t Buffer_8_can_take_input_for_dma : 1;/*!< Stores 1 if there is space available in buffer 8 for the size
                                                     requested by dma, else stores 0                                           */
      __IOM uint32_t Buffer_8_output_ready_for_dma : 1;/*!< Stores 1 if there is data available in buffer 8 for the size
                                                     requested by dma, else stores 0                                           */
      __IOM uint32_t Buffer_12_can_take_input_for_dma : 1;/*!< Stores 1 if there is space available in buffer 4 and 8 combined
                                                     for the size requested by dma, else stores 0                              */
      __IOM uint32_t Buffer_12_output_ready_for_dma : 1;/*!< Stores 1 if there is data available in buffer 4 and 8 combined
                                                     for the size requested by dma, else stores 0                              */
            uint32_t            : 18;
    } GPIO_BUFFER_STATUS_b;
  } ;
  __IM  uint32_t  RESERVED8;
  __IOM uint32_t  GPIO_BUFFER_2_CLOCK_PRESCALAR;/*!< To set the prescalar for buffer 2 internal clock                          */
  __IM  uint32_t  RESERVED9;
  __IOM uint32_t  GPIO_BUFFER_4_CLOCK_PRESCALAR;/*!< To set the prescalar for buffer 4 internal clock                          */
  __IM  uint32_t  RESERVED10;
  __IOM uint32_t  GPIO_BUFFER_8_CLOCK_PRESCALAR;/*!< To set the prescalar for buffer 8 internal clock                          */
  __IM  uint32_t  RESERVED11;
  __IOM uint32_t  GPIO_BUFFER_4_8_CLOCK_PRESCALAR;/*!< To set the prescalar for buffer 12 internal clock                       */
  __IM  uint32_t  RESERVED12;
  __IOM Data  GPIO_BUFFER_2_DATA;           /*!< To read the data from buffer 2                                            */
  __IM  uint32_t  RESERVED13;
  __IOM Data  GPIO_BUFFER_4_DATA;           /*!< To read the data from buffer 4                                            */
  __IM  uint32_t  RESERVED14;
  __IOM Data GPIO_BUFFER_8_DATA;           /*!< To read the data from buffer 8                                            */
  __IM  uint32_t  RESERVED15;
  __IOM Buf_4_8_Data  GPIO_BUFFER_4_8_DATA;         /*!< To read the data from buffer 12                                           */
} GPIO_Type;                                    /*!< Size = 140 (0x8c)                                                         */


/**
  * @brief General purpose IO. 13 GPIOs are pinmuxed with secondary level enable (GPIO_PINMUX)
  */

typedef struct {                                /*!< GPIO_PINMUX Structure                                                     */
  __IOM uint32_t  GPIO_DIRECTION;               /*!< Select the direction of the GPIOs. Each bit position corresponds
                                                     to the respective GPIO pin. 0 - Output, 1 - Input                         */
  __IM  uint32_t  RESERVED;
  __IOM uint32_t  GPIO_DATA;                    /*!< Contains the data to be sent out if the GPIO pin is configured
                                                     as output and the data recieved when configured as input.
                                                     Each bit position corresponds to the respective GPIO pin.                 */
  __IM  uint32_t  RESERVED1;
  __IOM uint32_t  GPIO_SET;                     /*!< To set the respective GPIO pins                                           */
  __IM  uint32_t  RESERVED2;
  __IOM uint32_t  GPIO_CLEAR;                   /*!< To clear the respective GPIO pins                                         */
  __IM  uint32_t  RESERVED3;
  __IOM uint32_t  GPIO_TOGGLE;                  /*!< To invert the respective GPIO pins                                        */
  __IM  uint32_t  RESERVED4[3];
  __IOM uint32_t  GPIO_INTR;                    /*!< To enable the interrupt of respective GPIO pins                           */
  __IM  uint32_t  RESERVED5;
  __IOM uint32_t  GPIO_PULLUP_CONFIG;           /*!< To enable the interrupt of respective GPIO pins                           */
} GPIO_PINMUX_Type;     

static void __iomem *mindgrove_get_base(
        struct mindgrove_gpio *mg,
        unsigned int offset,
        unsigned int *bit)
{
    if (offset < 32) {
        *bit = offset;
        return mg->reg_base;
    }

    *bit = offset - 32;
    return mg->pinmux_reg_base;
}


// #define GPIO_PINMUX_BASE            0x00040300UL
// #define GPIO_BASE                   0x00040200UL
// #define IORESOURCE_MEM         GPIO_BASE
#define GPIO_LINE_DIRECTION_IN	1
#define GPIO_LINE_DIRECTION_OUT	0
// #define GPIO_REG ((GPIO_Type*)(GPIO_BASE))
// //in the physical mapping have to be changed in probe implementation -> dummy for now
// #define GPIO_PINMUX_REG ((GPIO_PINMUX_Type*)(GPIO_PINMUX_BASE))

// #define GPIO_REG_BASE GPIO_BASE
// #define GPIO_PINMUX_REG_BASE GPIO_PINMUX_BASE

struct mindgrove_gpio {
	void __iomem *reg_base;
	void __iomem *pinmux_reg_base;
	struct gpio_chip gc;
};

static int mindgrove_gpio_get_direction(struct gpio_chip *gc, unsigned offset)
{
	struct mindgrove_gpio *mindgrove = gpiochip_get_data(gc);
  struct GPIO_Type __iomem *gpio;
  struct GPIO_PINMUX_Type __iomem *pinmux;
	uint32_t direction;
  unsigned int bit;

  if (offset < 32) {
        gpio = (struct GPIO_Type __iomem *)mindgrove->reg_base;
        bit = offset;
        direction = readl(&gpio->GPIO_DIRECTION);
  } else {
        pinmux = (struct GPIO_PINMUX_Type __iomem *)
                    mindgrove->pinmux_reg_base;
        bit = offset - 32;
        direction = readl(&pinmux->GPIO_DIRECTION);
  }

    return (direction & BIT(bit)) ?
            GPIO_LINE_DIRECTION_IN :
            GPIO_LINE_DIRECTION_OUT;
}


static int mindgrove_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	struct mindgrove_gpio *mindgrove = gpiochip_get_data(gc);
  struct GPIO_Type __iomem *gpio;
  struct GPIO_PINMUX_Type __iomem *pinmux;
	if (offset < 32) {
    gpio = (struct GPIO_Type __iomem *)mindgrove->reg_base;
    writel(readl(&(gpio->GPIO_DIRECTION))& ~BIT(offset),
            &gpio->GPIO_DIRECTION);
	} else {
    pinmux = (struct GPIO_PINMUX_Type __iomem *)mindgrove->pinmux_reg_base;
    writel(readl(pinmux->GPIO_DIRECTION) & ~BIT(offset-32),
		        &pinmux->GPIO_DIRECTION);
	}
  //setting the bit as 0 for input configuration.

	return 0;
}

static int mindgrove_gpio_direction_output(struct gpio_chip *gc, unsigned offset, int value)
{
	struct mindgrove_gpio *mindgrove = gpiochip_get_data(gc);
  struct GPIO_Type __iomem *gpio;
  struct GPIO_PINMUX_Type __iomem *pinmux;
	if (offset < 32) {
    gpio = (struct GPIO_Type __iomem *)mindgrove->reg_base;
    writel((readl(&gpio->GPIO_DIRECTION) | BIT(offset)),
            &gpio->GPIO_DIRECTION);
		if (value)
      writel((readl(&(gpio->GPIO_SET))|BIT(offset)),&gpio->GPIO_SET);
		else
      writel((readl(&(gpio->GPIO_CLEAR))|BIT(offset)),&gpio->GPIO_CLEAR);
	} else {
    pinmux = (struct GPIO_PINMUX_Type __iomem *)mindgrove->pinmux_reg_base;
    writel((readl(&pinmux->GPIO_DIRECTION) | BIT(offset-32)),
              &pinmux->GPIO_DIRECTION);
		if (value)
      writel((readl(&(pinmux->GPIO_SET))|BIT(offset-32)),&pinmux->GPIO_SET);
		else
      writel((readl(&(pinmux->GPIO_CLEAR))|BIT(offset-32)),&pinmux->GPIO_CLEAR);
	}

	return 0;
}

static int mindgrove_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct mindgrove_gpio *mindgrove = gpiochip_get_data(gc);
  struct GPIO_Type __iomem *gpio;
  struct GPIO_PINMUX_Type __iomem *pinmux;
	u32 data; //some builtin thing that it supports

	if (offset < 32) {
    gpio = (struct GPIO_Type __iomem *)mindgrove->reg_base;
    data = readl(&gpio->GPIO_DATA);
    return !!(data & BIT(offset));
	} else {
    pinmux = (struct GPIO_PINMUX_Type __iomem *)mindgrove->pinmux_reg_base;
    data=readl(&pinmux->GPIO_DATA);
    return !!(data & BIT(offset-32));
	}
}

static void mindgrove_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct mindgrove_gpio *mindgrove = gpiochip_get_data(gc);
  struct GPIO_Type __iomem *gpio;
  struct GPIO_PINMUX_Type __iomem *pinmux;

	if (offset < 32) {
    gpio = (struct GPIO_Type __iomem *)mindgrove->reg_base;
		if (value)
      writel((BIT(offset)|readl(&(gpio->GPIO_SET))),&gpio->GPIO_SET);
		else
			writel((BIT(offset)|readl(&(gpio->GPIO_CLEAR))), &gpio->GPIO_CLEAR);
	} else {
    pinmux = (struct GPIO_PINMUX_Type __iomem *)mindgrove->pinmux_reg_base;
		if (value)
			writel((readl(&pinmux->GPIO_SET)|BIT(offset-32)), &pinmux->GPIO_SET);
		else
			writel((readl(&pinmux->GPIO_CLEAR)|BIT(offset-32)), &pinmux->GPIO_CLEAR);
	}
}

/*

static int mindgrove_gpio_irq_type(struct gpio_chip *gc, unsigned offset, unsigned int type)
{
	struct mindgrove_gpio *mindgrove = gpiochip_get_data(gc);

	if (offset < 32) {
		if (type & IRQ_TYPE_EDGE_FALLING)
			writel(readl(mindgrove->reg_base + GPIO_INTR) & ~BIT(offset),
			       mindgrove->reg_base + GPIO_INTR);
		else
			writel(readl(mindgrove->reg_base + GPIO_INTR) | BIT(offset),
			       mindgrove->reg_base + GPIO_INTR);
	} else {
		if (type & IRQ_TYPE_EDGE_FALLING)
			writel(readl(mindgrove->pinmux_reg_base + GPIO_INTR) & ~BIT(offset),
			       mindgrove->pinmux_reg_base + GPIO_INTR);
		else
			writel(readl(mindgrove->pinmux_reg_base + GPIO_INTR) | BIT(offset),
			       mindgrove->pinmux_reg_base + GPIO_INTR);
	}

	return 0;
}*/

static int mindgrove_gpio_probe(struct platform_device *pdev)
{
	struct mindgrove_gpio *mindgrove;
	struct resource *res; 
	int ret;

	mindgrove = devm_kzalloc(&pdev->dev, sizeof(*mindgrove), GFP_KERNEL);
	if (!mindgrove)
		return -ENOMEM;

    // I think IORESOURCE_MEM must point to the base address of gpio right?
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mindgrove->reg_base = devm_ioremap_resource(&pdev->dev, res);
    //not passing the base address directly and using linux address remap for it.
	if (IS_ERR(mindgrove->reg_base))
		return PTR_ERR(mindgrove->reg_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	mindgrove->pinmux_reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mindgrove->pinmux_reg_base))
		return PTR_ERR(mindgrove->pinmux_reg_base);

  mindgrove->gc.owner = THIS_MODULE;
	mindgrove->gc.label = "mindgrove-gpio";
	mindgrove->gc.parent = &pdev->dev;
	mindgrove->gc.base = -1;
	mindgrove->gc.ngpio = 45;
	mindgrove->gc.direction_input = mindgrove_gpio_direction_input;
	mindgrove->gc.direction_output = mindgrove_gpio_direction_output;
	mindgrove->gc.get_direction = mindgrove_gpio_get_direction;
	mindgrove->gc.get = mindgrove_gpio_get;
	mindgrove->gc.set = mindgrove_gpio_set;
	//mindgrove->gc.set_config = mindgrove_gpio_irq_type;

	ret = devm_gpiochip_add_data(&pdev->dev, &mindgrove->gc, mindgrove);
    //In intel they didnt add the overal struct also
	if (ret) {
		dev_err(&pdev->dev, "Failed to register GPIO chip\n");
		return ret;
	}

	platform_set_drvdata(pdev, mindgrove);

	return 0;
}

static const struct of_device_id mindgrove_gpio_of_match[] = {
	{ .compatible = "mindgrove,secure-iot-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mindgrove_gpio_of_match);

static struct platform_driver mindgrove_gpio_driver = {
	.driver = {
		.name = "mindgrove-gpio",
		.of_match_table = mindgrove_gpio_of_match,
	},
	.probe = mindgrove_gpio_probe,
};

module_platform_driver(mindgrove_gpio_driver);

MODULE_AUTHOR("Biancaa Ramesh");
MODULE_DESCRIPTION("Mindgrove Silicon GPIO driver");
MODULE_LICENSE("GPL");


