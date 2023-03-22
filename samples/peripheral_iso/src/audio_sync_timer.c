/*
 *  Copyright (c) 2021, PACKETCRAFT, INC.
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <nrfx_dppi.h>
#include <nrfx_ipc.h>

#include "nrf5340_audio_common.h"
#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_sync_timer, 4);

#define AUDIO_SYNC_TIMER_INSTANCE_NUMBER 1

#define AUDIO_SYNC_TIMER_NET_APP_IPC_EVT NRF_IPC_EVENT_RECEIVE_4

const nrfx_timer_t audio_sync_timer_instance =
	NRFX_TIMER_INSTANCE(AUDIO_SYNC_TIMER_INSTANCE_NUMBER);

static uint8_t dppi_channel_timer_clear;

extern int test_led_state;
static nrfx_timer_config_t cfg = { .frequency = NRF_TIMER_FREQ_1MHz,
				   .mode = NRF_TIMER_MODE_TIMER,
				   .bit_width = NRF_TIMER_BIT_WIDTH_32,
				   .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
				   .p_context = &test_led_state };

static void event_handler(nrf_timer_event_t event_type, void *ctx)
{
	int led_state = *(int *)ctx;
	dk_set_led(DK_LED2, led_state);
}

/**
 * @brief Initialize audio sync timer
 *
 * @note Clearing of the nRF5340 APP core sync
 * timer is initialized here. The sync timers on
 * APP core and NET core are cleared at exactly
 * the same time using an IPC signal sent from
 * the NET core. This makes the two timers
 * synchronized.
 *
 * @param unused Unused
 *
 * @return 0 if successful, error otherwise
 */
static int audio_sync_timer_init(const struct device *unused)
{
	nrfx_err_t ret;

	ARG_UNUSED(unused);

	ret = nrfx_timer_init(&audio_sync_timer_instance, &cfg, event_handler);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx timer init error - Return value: %d", ret);
		return ret;
	}
	IRQ_CONNECT(TIMER1_IRQn, 4, nrfx_timer_1_irq_handler, NULL, 0);
	nrfx_timer_enable(&audio_sync_timer_instance);

	/* Initialize functionality for synchronization between APP and NET core */
	ret = nrfx_dppi_channel_alloc(&dppi_channel_timer_clear);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel alloc error (timer clear) - Return value: %d", ret);
		return ret;
	}

	nrf_ipc_publish_set(NRF_IPC, AUDIO_SYNC_TIMER_NET_APP_IPC_EVT, dppi_channel_timer_clear);
	nrf_timer_subscribe_set(audio_sync_timer_instance.p_reg, NRF_TIMER_TASK_CLEAR,
				dppi_channel_timer_clear);
	ret = nrfx_dppi_channel_enable(dppi_channel_timer_clear);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel enable error (timer clear) - Return value: %d", ret);
		return ret;
	}

	LOG_DBG("Audio sync timer initialized");

	return 0;
}

SYS_INIT(audio_sync_timer_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
