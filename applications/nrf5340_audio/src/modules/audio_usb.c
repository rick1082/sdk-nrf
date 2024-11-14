/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "audio_usb.h"

#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_audio.h>
#include <data_fifo.h>

#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_usb, CONFIG_MODULE_AUDIO_USB_LOG_LEVEL);

#define USB_FRAME_SIZE_STEREO                                                                      \
	(((CONFIG_AUDIO_SAMPLE_RATE_HZ * CONFIG_AUDIO_BIT_DEPTH_OCTETS) / 1000) * 2)

static struct data_fifo *fifo_tx;
static struct data_fifo *fifo_rx;

NET_BUF_POOL_FIXED_DEFINE(pool_out, CONFIG_FIFO_FRAME_SPLIT_NUM, USB_FRAME_SIZE_STEREO, 8,
			  net_buf_destroy);

static uint32_t rx_num_overruns;
static bool rx_first_data;
static bool tx_first_data;

#if (CONFIG_STREAM_BIDIRECTIONAL)
static uint32_t tx_num_underruns;

static void data_write(const struct device *dev)
{
	int ret;

	if (fifo_tx == NULL) {
		return;
	}

	void *data_out;
	size_t data_out_size;
	struct net_buf *buf_out;

	buf_out = net_buf_alloc(&pool_out, K_NO_WAIT);

	ret = data_fifo_pointer_last_filled_get(fifo_tx, &data_out, &data_out_size, K_NO_WAIT);
	if (ret) {
		tx_num_underruns++;
		if ((tx_num_underruns % 100) == 1) {
			LOG_WRN("USB TX underrun. Num: %d", tx_num_underruns);
		}
		net_buf_unref(buf_out);

		return;
	}

	memcpy(buf_out->data, data_out, data_out_size);
	data_fifo_block_free(fifo_tx, data_out);

	if (data_out_size == usb_audio_get_in_frame_size(dev)) {
		ret = usb_audio_send(dev, buf_out, data_out_size);
		if (ret) {
			LOG_WRN("USB TX failed, ret: %d", ret);
			net_buf_unref(buf_out);
		}

	} else {
		LOG_WRN("Wrong size write: %d", data_out_size);
	}

	if (!tx_first_data) {
		LOG_INF("USB TX first data sent.");
		tx_first_data = true;
	}
}
#endif /* (CONFIG_STREAM_BIDIRECTIONAL) */

static void data_received(const struct device *dev, struct net_buf *buffer, size_t size)
{

}

static void feature_update(const struct device *dev, const struct usb_audio_fu_evt *evt)
{

}

static const struct usb_audio_ops ops = {
	.data_received_cb = data_received,
	.feature_update_cb = feature_update,
#if (CONFIG_STREAM_BIDIRECTIONAL)
	.data_request_cb = data_write,
#endif /* (CONFIG_STREAM_BIDIRECTIONAL) */
};

int audio_usb_start(struct data_fifo *fifo_tx_in, struct data_fifo *fifo_rx_in)
{

	return 0;
}

void audio_usb_stop(void)
{

}

int audio_usb_disable(void)
{
	int ret;

	return 0;
}

int audio_usb_init(void)
{
	int ret;


	return 0;
}
