/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the STM32
 * implementation.
 */
#ifndef __PLATFORM_H
#define __PLATFORM_H

#include "gpio.h"
#include "timing.h"
#include "timing_stm32.h"

#define PLATFORM_HAS_TRACESWO   1
#ifdef ENABLE_DEBUG
#define PLATFORM_HAS_DEBUG
#define USBUART_DEBUG
#endif
#define BOARD_IDENT             "Black Magic Probe"
#define BOARD_IDENT_DFU	        "Black Magic Probe (Upgrade)"
#define BOARD_IDENT_UPD	        "Black Magic Probe (DFU Upgrade)"
#define DFU_IDENT               "Black Magic Firmware Upgrade"
#define UPD_IFACE_STRING        "@Internal Flash   /0x08000000/8*001Kg"

/* Important pin mappings for STM32 implementation:
 *
 * LED0 = 	PC13	(IDLE/RUN)
 * LED1 = 	PA1 	(UART)
 *
 * We re use on board jtag/swd port
 * SRST_OUT = 	PB4 (output)
 * TDI = 	PA15 (output)
 * TMS = 	PA13 (input/output for SWDIO)
 * TCK = 	PA14 (output SWCLK)
 * TDO = 	PB3 (input)
 *
 * Force DFU mode button: PA0
 */

/* Hardware definitions... */
// JTAG
#define JTAG_PORT 	GPIOA
#define TDI_PORT	JTAG_PORT
#define TMS_PORT	JTAG_PORT
#define TCK_PORT	JTAG_PORT
#define TDO_PORT	GPIOB
#define TDI_PIN		GPIO15
#define TMS_PIN		GPIO13
#define TCK_PIN		GPIO14
#define TDO_PIN		GPIO3

#define SWDIO_PORT 	JTAG_PORT
#define SWCLK_PORT 	JTAG_PORT
#define SWDIO_PIN	TMS_PIN
#define SWCLK_PIN	TCK_PIN

// SWD
#define TRST_PORT	GPIOB
#define TRST_PIN	GPIO4
#define SRST_PORT	TRST_PORT
#define SRST_PIN	TRST_PIN

#define LED_PORT	GPIOC
#define LED_PORT_UART	GPIOA
#define LED_0		GPIO13
#define LED_1       GPIO1
#define LED_UART	LED_1
#define LED_IDLE_RUN            LED_0

#define TMS_SET_MODE() do { \
	gpio_set_mode(TMS_PORT, GPIO_MODE_OUTPUT_50_MHZ, \
	              GPIO_CNF_OUTPUT_PUSHPULL, TMS_PIN); \
} while(0)
#define SWDIO_MODE_FLOAT() do { \
	gpio_set_mode(SWDIO_PORT, GPIO_MODE_INPUT, \
	              GPIO_CNF_INPUT_FLOAT, SWDIO_PIN); \
} while(0)
#define SWDIO_MODE_DRIVE() do { \
	gpio_set_mode(SWDIO_PORT, GPIO_MODE_OUTPUT_50_MHZ, \
	              GPIO_CNF_OUTPUT_PUSHPULL, SWDIO_PIN); \
} while(0)
#define UART_PIN_SETUP() do { \
	gpio_set_mode(USBUSART_PORT, GPIO_MODE_OUTPUT_2_MHZ, \
	              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, USBUSART_TX_PIN); \
} while(0)

#define USB_DRIVER st_usbfs_v1_usb_driver
#define USB_IRQ    NVIC_USB_LP_CAN_RX0_IRQ
#define USB_ISR    usb_lp_can_rx0_isr
/* Interrupt priorities.  Low numbers are high priority.
 * For now USART1 preempts USB which may spin while buffer is drained.
 * TIM2 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB             (2 << 4)
#define IRQ_PRI_USBUSART        (1 << 4)
#define IRQ_PRI_USBUSART_TIM    (3 << 4)

#if (SWO_TRACE_MODE == 1)
#define IRQ_PRI_TRACE           (0 << 4)

#define USBUSART USART1
#define USBUSART_CR1 USART1_CR1
#define USBUSART_IRQ NVIC_USART1_IRQ
#define USBUSART_CLK RCC_USART1
#define USBUSART_PORT GPIOA
#define USBUSART_TX_PIN GPIO9
#define USBUSART_ISR usart1_isr
#define USBUSART_TIM TIM4
#define USBUSART_TIM_CLK_EN() rcc_periph_clock_enable(RCC_TIM4)
#define USBUSART_TIM_IRQ NVIC_TIM4_IRQ
#define USBUSART_TIM_ISR tim4_isr

#define TRACE_TIM TIM2
#define TRACE_TIM_CLK_EN() rcc_periph_clock_enable(RCC_TIM2)
#define TRACE_IRQ   NVIC_TIM2_IRQ
#define TRACE_ISR   tim2_isr

#else
#define NUM_TRACE_PACKETS		(128)		/* This is an 8K buffer */

#define IRQ_PRI_SWO_DMA			(1 << 4)

#define USBUSART USART2
#define USBUSART_CR1 USART2_CR1
#define USBUSART_IRQ NVIC_USART2_IRQ
#define USBUSART_CLK RCC_USART2
#define USBUSART_PORT GPIOA
#define USBUSART_TX_PIN GPIO2
#define USBUSART_ISR usart2_isr
#define USBUSART_TIM TIM4
#define USBUSART_TIM_CLK_EN() rcc_periph_clock_enable(RCC_TIM4)
#define USBUSART_TIM_IRQ NVIC_TIM4_IRQ
#define USBUSART_TIM_ISR tim4_isr

/* On F103, only USART1 is on AHB2 and can reach 4.5 MBaud at 72 MHz.*/
#define SWO_UART				USART1
#define SWO_UART_DR				USART1_DR
#define SWO_UART_CLK			RCC_USART1
#define SWO_UART_PORT			GPIOA
#define SWO_UART_RX_PIN			GPIO10

/* This DMA channel is set by the USART in use */
#define SWO_DMA_BUS				DMA1
#define SWO_DMA_CLK				RCC_DMA1
#define SWO_DMA_CHAN			DMA_CHANNEL5
#define SWO_DMA_IRQ				NVIC_DMA1_CHANNEL5_IRQ
#define SWO_DMA_ISR(x)			dma1_channel5_isr(x)

#endif

#ifdef ENABLE_DEBUG
extern bool debug_bmp;
int usbuart_debug_write(const char *buf, size_t len);

#define DEBUG printf
#else
#define DEBUG(...)
#endif

#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)	{gpio_set_val(LED_PORT, LED_IDLE_RUN, !state);}
#define SET_ERROR_STATE(state)

/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define snprintf sniprintf
#define vasprintf vasiprintf

#endif
