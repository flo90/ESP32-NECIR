/*  Copyright (C) 2019  Florian Menne
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <driver/rmt.h>
#include "necir.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"

#include <stdbool.h>

#define TAG "NECIR"

#ifndef CONFIG_NECIR_RMT_RXCHANNEL
#define CONFIG_NECIR_RMT_RXCHANNEL 0
#endif

#ifndef CONFIG_NECIR_RMT_RXGPIO
#define CONFIG_NECIR_RMT_RXGPIO 19
#endif

#ifndef CONFIG_NECIR_RMT_RXMEM
#define CONFIG_NECIR_RMT_RXMEM 1
#endif

#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)
#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */

#define NEC_HEADER_HIGH_US 9000
#define NEC_HEADER_LOW_US 4500

#define NEC_HEADER_HIGH_TICK (NEC_HEADER_HIGH_US / 10 * RMT_TICK_10_US)
#define NEC_HEADER_LOW_TICK (NEC_HEADER_LOW_US / 10 * RMT_TICK_10_US)


#define NEC_BITX_HIGH_US 560
#define NEC_BIT1_LOW_US 1690
#define NEC_BIT0_LOW_US 560

#define NEC_BITX_HIGH_TICK (NEC_BITX_HIGH_US / 10 * RMT_TICK_10_US)
#define NEC_BIT1_LOW_TICK (NEC_BIT1_LOW_US / 10 * RMT_TICK_10_US)
#define NEC_BIT0_LOW_TICK (NEC_BIT0_LOW_US / 10 * RMT_TICK_10_US)

#define NEC_BIT_HEADER_MARGIN_US 250
#define NEC_BIT_HEADER_MARGIN_TICK (NEC_BIT_HEADER_MARGIN_US / 10 * RMT_TICK_10_US)

#define NEC_BIT_MARGIN_US 250
#define NEC_BIT_MARGIN_TICK (NEC_BIT_MARGIN_US / 10 * RMT_TICK_10_US)

enum {
	STATE_FIRST_TIME_HEADER,
	STATE_ADDR,
	STATE_ADDR_INV,
	STATE_CMD,
	STATE_CMD_INV,
	STATE_FINAL,
	STATE_REPEAT,
};

void rx_task(void* args);
void parse_nec(rmt_item32_t* items, size_t size);

static inline bool necBit(rmt_item32_t* item);
static inline bool necHeader(rmt_item32_t* item);
static inline bool nextState(uint8_t* bit);

void necir_init()
{
	rmt_config_t rmt_rx;
	rmt_rx.channel = CONFIG_NECIR_RMT_RXCHANNEL;
	rmt_rx.gpio_num = CONFIG_NECIR_RMT_RXGPIO;
	rmt_rx.clk_div = RMT_CLK_DIV;
	rmt_rx.mem_block_num = CONFIG_NECIR_RMT_RXMEM;
	rmt_rx.rmt_mode = RMT_MODE_RX;
	rmt_rx.rx_config.filter_en = true;
	rmt_rx.rx_config.filter_ticks_thresh = 100;
	rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
	rmt_config(&rmt_rx);
	rmt_driver_install(rmt_rx.channel, 1000, 0);

	xTaskCreate(rx_task, "necir_rx_task", 4096, NULL, 10, NULL);
}

void rx_task(void* args)
{
	int channel = CONFIG_NECIR_RMT_RXCHANNEL;
	RingbufHandle_t rb = NULL;

	rmt_get_ringbuf_handle(channel, &rb);
	rmt_rx_start(channel, 1);

	while(rb) {
		size_t rx_size = 0;

		rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);

		if(item)
		{
			parse_nec(item, rx_size / 4);
			vRingbufferReturnItem(rb, (void*) item);
		}
	}
	vTaskDelete(NULL);
}

void parse_nec(rmt_item32_t* items, size_t size)
{
	static uint32_t state = STATE_FIRST_TIME_HEADER;
	static uint8_t bit;

	static uint8_t addr;
	static uint8_t addrInv;

	static uint8_t cmd;
	static uint8_t cmdInv;
	static uint32_t repeat;

	size_t i = 0;

	//Detect header
	if( necHeader(items) )
	{
		//ESP_LOGI(TAG, "HEADER DETECTED");
		state = STATE_ADDR;

		bit = 0;

		addr = 0;
		addrInv = 0;

		cmd = 0;
		cmdInv = 0;

		repeat = 0;

		i = 1;
	}

	for(; i < size; i++)
	{
		//ESP_LOGI(TAG, "%d %d %d %d", items[i].level0, items[i].duration0, items[i].level1, items[i].duration1);


		switch(state) {
		case STATE_FIRST_TIME_HEADER:
			break;

		case STATE_ADDR:
			if(necBit(&items[i]))
				addr |= (1<<bit);

			if(nextState(&bit))
				state = STATE_ADDR_INV;

			break;

		case STATE_ADDR_INV:
			if(necBit(&items[i]))
				addrInv |= (1<<bit);

			if(nextState(&bit))
				state = STATE_CMD;

			break;

		case STATE_CMD:
			if(necBit(&items[i]))
				cmd |= (1<<bit);

			if(nextState(&bit))
				state = STATE_CMD_INV;

			break;

		case STATE_CMD_INV:
			if(necBit(&items[i]))
				cmdInv |= (1<<bit);

			if(nextState(&bit))
			{
				if(cmd == ((uint8_t)~cmdInv))
					necir_callback(addr | addrInv<<8, cmd, repeat);
				state = STATE_REPEAT;
			}


			break;

		case STATE_FINAL:

			break;

		case STATE_REPEAT:
			repeat++;
			if(cmd == ((uint8_t)~cmdInv))
				necir_callback(addr | addrInv<<8, cmd, repeat);
			break;

		default:
			break;
		}
	}
}

static inline bool necBit(rmt_item32_t* item)
{
	if(
			item->level0 == 0 &&
			item->level1 == 1 &&

			item->duration0 >= NEC_BITX_HIGH_TICK - NEC_BIT_MARGIN_TICK &&
			item->duration0 < NEC_BITX_HIGH_TICK + NEC_BIT_MARGIN_TICK &&

			item->duration1 >= NEC_BIT1_LOW_TICK - NEC_BIT_MARGIN_TICK &&
			item->duration1 < NEC_BIT1_LOW_TICK + NEC_BIT_MARGIN_TICK
	)
	{
		return true;
	}
	return false;
}

static inline bool necHeader(rmt_item32_t* item)
{
	if(
			item->level0 == 0 &&
			item->level1 == 1 &&

			item->duration0 >= NEC_HEADER_HIGH_TICK - NEC_BIT_HEADER_MARGIN_TICK &&
			item->duration0 < NEC_HEADER_HIGH_TICK + NEC_BIT_HEADER_MARGIN_TICK &&

			item->duration1 >= NEC_HEADER_LOW_TICK - NEC_BIT_HEADER_MARGIN_TICK &&
			item->duration1 < NEC_HEADER_LOW_TICK + NEC_BIT_HEADER_MARGIN_TICK
	)
	{
		return true;
	}
	return false;
}

static inline bool nextState(uint8_t* bit)
{
	if((*bit) >= 7)
	{
		*bit = 0;
		return true;
	}
	(*bit)++;
	return false;
}


__attribute__((weak)) void necir_callback(uint16_t addr, uint8_t cmd, uint32_t repeat)
{
	ESP_LOGI(TAG, "ADDR: %.2x ADDR_INV %.2x CMD: %.2x Repeat: %d", addr&0xFF, (addr>>8)&0xFF, cmd, repeat);
}

