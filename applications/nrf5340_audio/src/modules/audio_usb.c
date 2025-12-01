/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "audio_usb.h"

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_audio.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <audio_defines.h>

#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_usb, CONFIG_MODULE_AUDIO_USB_LOG_LEVEL);

static struct k_msgq *audio_q_tx;
static struct k_msgq *audio_q_rx;

NET_BUF_POOL_FIXED_DEFINE(pool_in, CONFIG_FIFO_FRAME_SPLIT_NUM, USB_BLOCK_SIZE_STEREO,
			  sizeof(struct audio_metadata), NULL);

static uint32_t rx_num_overruns;
static bool rx_first_data;
static bool tx_first_data;

int audio_usb_start(struct k_msgq *audio_q_tx_in, struct k_msgq *audio_q_rx_in)
{
	if (audio_q_tx_in == NULL || audio_q_rx_in == NULL) {
		return -EINVAL;
	}

	audio_q_tx = audio_q_tx_in;
	audio_q_rx = audio_q_rx_in;

	return 0;
}

void audio_usb_stop(void)
{
	rx_first_data = false;
	tx_first_data = false;
	audio_q_tx = NULL;
	audio_q_rx = NULL;
}

int audio_usb_disable(void)
{
	int ret;

	audio_usb_stop();

	return 0;
}

int audio_usb_init(void)
{
	int ret;

	LOG_INF("Ready for USB host to send/receive.");

	return 0;
}
