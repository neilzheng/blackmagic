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

#include "general.h"
#include "cdcacm.h"
#include "usbuart.h"
#include "morse.h"

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/f1/adc.h>

/* Pins PB[7:5] are used to detect hardware revision.
 * 000 - Original production build.
 * 001 - Mini production build.
 * 010 - Mini V2.0e and later.
 */
int platform_hwversion(void)
{
	static int hwversion = -1;
	uint16_t hwversion_pins = GPIO2;
	uint16_t unused_pins = hwversion_pins ^ 0xFFFF;

	/* Only check for version if this is the first time. */
	if (hwversion == -1) {
		/* Configure the hardware version pins as input pull-up/down */
		gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
				GPIO_CNF_INPUT_PULL_UPDOWN,
				hwversion_pins);

		/* Get all pins that are pulled low in hardware.
		 * This also sets all the "unused" pins to 1.
		 */
		uint16_t pins_negative = gpio_get(GPIOB, hwversion_pins) | unused_pins;

		/* Hardware version is the id defined by the pins that are
		 * asserted low or high by the hardware. This means that pins
		 * that are left floating are 0 and those that are either
		 * pulled high or low are 1.
		 */
		hwversion = (((pins_negative) ^ 0xFFFF) & hwversion_pins) >> 5;
	}

	return hwversion;
}

void platform_init(void)
{
	SCS_DEMCR |= SCS_DEMCR_VC_MON_EN;
#ifdef ENABLE_DEBUG
	void initialise_monitor_handles(void);
	initialise_monitor_handles();
#endif

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_CRC);
	rcc_periph_clock_enable(RCC_AFIO);

    /* Unmap JTAG & SWD Pins so we can reuse as GPIO */
    /* remap tim2 to PA15/PB3/PB10/PB11 */
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, AFIO_MAPR_TIM2_REMAP_FULL_REMAP);

	/* Setup JTAG GPIO ports */
	gpio_set_mode(TMS_PORT, GPIO_MODE_OUTPUT_10_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, TMS_PIN);
	gpio_set_mode(TCK_PORT, GPIO_MODE_OUTPUT_10_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, TCK_PIN);

	gpio_set_mode(LED_PORT_UART, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL,
			LED_UART);
	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL,
			LED_IDLE_RUN);
	gpio_set(LED_PORT, LED_IDLE_RUN);
    gpio_clear(LED_PORT_UART, LED_UART);

	/* Enable SRST output. Original uses a NPN to pull down, so setting the
	 * output HIGH asserts. Mini is directly connected so use open drain output
	 * and set LOW to assert.
	 */
	platform_srst_set_val(false);
	gpio_set_mode(SRST_PORT, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL,
			SRST_PIN);

	/* Relocate interrupt vector table here */
	extern int vector_table;
	SCB_VTOR = (uint32_t)&vector_table;

	platform_timing_init();
	cdcacm_init();

	usbuart_init();

}

void platform_srst_set_val(bool assert)
{
	gpio_set_val(TMS_PORT, TMS_PIN, 1);
	gpio_set_val(SRST_PORT, SRST_PIN, !assert);
	if (assert) {
		for(int i = 0; i < 10000; i++) asm("nop");
	}
}

bool platform_srst_get_val(void)
{
	return gpio_get(SRST_PORT, SRST_PIN) == 0;
}

const char *platform_target_voltage(void)
{
	return "unknown";
}

void platform_request_boot(void)
{
	uint32_t crl = GPIOA_CRL;
    rcc_periph_clock_enable(RCC_GPIOA);
	/* Assert bootloader marker.
	 * Enable Pull on GPIOA1. We don't rely on the external pin
	 * really pulled, but only on the value of the CNF register
	 * changed from the reset value
	 */
	crl &= 0xffffff0f;
	crl |= 0x80;
	GPIOA_CRL = crl;
	SCB_VTOR = 0;
    gpio_clear(GPIOA, GPIO1);
}
