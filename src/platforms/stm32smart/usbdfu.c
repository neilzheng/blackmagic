/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2013 Gareth McMullin <gareth@blacksphere.co.nz>
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

#include <string.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/scb.h>

#include "usbdfu.h"
#include "platform.h"

uint32_t app_address = 0x08002000;
int dfu_activity_counter = 0;

void dfu_detach(void)
{
	/* USB device must detach, we just reset... */
	scb_reset_system();
}

int main(void)
{
    bool nbtn;

    rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_PULL_UPDOWN , GPIO0);
    gpio_set(GPIOA, GPIO0);
    for (int i=0; i<10000; i++) {
        nbtn = !!gpio_get(GPIOA, GPIO0);
    }
    if (((GPIOA_CRL & 0x40) == 0x40) && nbtn) {
        dfu_jump_app_if_valid();
    }
	dfu_protect(DFU_MODE);

    rcc_clock_setup_in_hse_8mhz_out_72mhz();
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(900000);

    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
	/* Configure USB related clocks and pins. */
	rcc_periph_clock_enable(RCC_USB);

	systick_interrupt_enable();
	systick_counter_enable();

	/* Configure the LED pins. */
	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, LED_IDLE_RUN);
	gpio_set_mode(LED_PORT_UART, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, LED_UART);
	gpio_set(LED_PORT, LED_IDLE_RUN);
    gpio_clear(LED_PORT_UART, LED_UART);

	dfu_init(&st_usbfs_v1_usb_driver, DFU_MODE);

	dfu_main();
}

void dfu_event(void)
{
	/* If the counter was at 0 before we should reset LED status. */
	if (dfu_activity_counter == 0) {
		gpio_set(LED_PORT, LED_IDLE_RUN);
        gpio_clear(LED_PORT_UART, LED_UART);
	}

	/* Prevent the sys_tick_handler from blinking leds for a bit. */
	dfu_activity_counter = 10;

	/* Toggle the DFU activity LED. */
	gpio_toggle(LED_PORT, LED_IDLE_RUN);
}

void sys_tick_handler(void)
{
	static int count = 0;
	static bool reset = true;

	/* Run the LED show only if there is no DFU activity. */
	if (dfu_activity_counter != 0) {
		dfu_activity_counter--;
		reset = true;
	} else {
		if (reset) {
			gpio_set(LED_PORT, LED_IDLE_RUN);
            gpio_clear(LED_PORT_UART, LED_UART);
			count = 0;
			reset = false;
		}

		switch (count) {
		case 1:
			gpio_toggle(LED_PORT_UART, LED_UART);
			count++;
			break;
		case 2:
			gpio_toggle(LED_PORT, LED_IDLE_RUN);
			count=0;
			break;
		}
	}
}

