/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "audio_usb.h"
#include <sample_usbd.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_uac2.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <audio_defines.h>

#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_usb, CONFIG_MODULE_AUDIO_USB_LOG_LEVEL);

static struct k_msgq *audio_q_tx;
static struct k_msgq *audio_q_rx;

/* USB Audio Data is downsampled from 48kHz to match broadcast preset when receiving data */
#define USB_SAMPLE_RATE           48000
#define USB_DOWNSAMPLE_RATE       BROADCAST_SAMPLE_RATE
#define USB_FRAME_DURATION_US     1000
#define USB_SAMPLE_CNT            ((USB_FRAME_DURATION_US * USB_SAMPLE_RATE) / USEC_PER_SEC)
#define USB_DOWNSSAMPLE_CNT       ((USB_FRAME_DURATION_US * USB_DOWNSAMPLE_RATE) / USEC_PER_SEC)
#define USB_BYTES_PER_SAMPLE      2
#define USB_CHANNELS              2
#define USB_MONO_FRAME_SIZE       (USB_SAMPLE_CNT * USB_BYTES_PER_SAMPLE)
/* The number of samples received may be USB_SAMPLE_CNT -+ 1 */
#define USB_MAX_MONO_FRAME_SIZE   ((USB_SAMPLE_CNT + 1) * USB_BYTES_PER_SAMPLE)
#define USB_STEREO_FRAME_SIZE     (USB_MONO_FRAME_SIZE * USB_CHANNELS)
#define USB_MAX_STEREO_FRAME_SIZE (USB_MAX_MONO_FRAME_SIZE * USB_CHANNELS)

NET_BUF_POOL_FIXED_DEFINE(pool_in, CONFIG_FIFO_FRAME_SPLIT_NUM, USB_BLOCK_SIZE_STEREO,
			  sizeof(struct audio_metadata), NULL);

K_MEM_SLAB_DEFINE_STATIC(usb_out_buf_pool, ROUND_UP(USB_MAX_STEREO_FRAME_SIZE, UDC_BUF_ALIGN), 3, UDC_BUF_ALIGN);

static uint32_t rx_num_overruns;
static bool rx_first_data;
static bool tx_first_data;
static bool terminal_enabled;

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
#define MICROPHONE_IN_TERMINAL_ID  UAC2_ENTITY_ID(DT_NODELABEL(in_terminal))
static uint8_t __aligned(UDC_BUF_ALIGN) data_buffer[ROUND_UP(96, UDC_BUF_GRANULARITY)] = {0};
/* USB consumer callback, called every 1ms, consumes data from ring-buffer */
static void uac2_sof_cb(const struct device *dev, void *user_data)
{
	usbd_uac2_send(dev, MICROPHONE_IN_TERMINAL_ID, data_buffer, 96);
}

static void *uac2_get_recv_buf(const struct device *dev, uint8_t terminal, uint16_t size,
			     void *user_data)
{
	void *buf = NULL;
	int ret;

	if (!terminal_enabled) {
		return NULL;
	}

	__ASSERT(size <= USB_MAX_STEREO_FRAME_SIZE, "%u was not <= %d", size,
		 USB_MAX_STEREO_FRAME_SIZE);

	ret = k_mem_slab_alloc(&usb_out_buf_pool, &buf, K_NO_WAIT);
	if (ret != 0) {
		printk("Failed to allocate buffer: %d\n", ret);
	}

	return buf;
}


static void uac2_data_recv_cb(const struct device *dev, uint8_t terminal, void *buf, uint16_t size,
			 void *user_data)
{
	int ret;
	struct net_buf *audio_block;

	if (!terminal_enabled || buf == NULL || size == 0U || audio_q_rx == NULL || size != 192) {
		k_mem_slab_free(&usb_out_buf_pool, buf);
		return;
	}

	/* RX FIFO can fill up due to re-transmissions or disconnect */
	/* Also check the availability in the pool, as the message queue might become
	 * available before the net_buf is unreferenced due to thread execution timing
	 */
	if (k_msgq_num_free_get(audio_q_rx) == 0 || pool_in.avail_count == 0) {
		struct net_buf *stale_usb_data;

		rx_num_overruns++;
		if ((rx_num_overruns % 100) == 1) {
			LOG_WRN("USB RX overrun. Num: %d", rx_num_overruns);
		}

		ret = k_msgq_get(audio_q_rx, (void *)&stale_usb_data, K_NO_WAIT);
		ERR_CHK(ret);

		net_buf_unref(stale_usb_data);
	}

	audio_block = net_buf_alloc(&pool_in, K_NO_WAIT);
	if (audio_block == NULL) {
		LOG_WRN("Out of RX buffers");
		k_mem_slab_free(&usb_out_buf_pool, buf);
		return;
	}

	net_buf_add_mem(audio_block, buf, size);

	//printk("Received %d data to input terminal %d\n", size, terminal);

	k_mem_slab_free(&usb_out_buf_pool, buf);

	struct audio_metadata *meta = net_buf_user_data(audio_block);

	meta->data_coding = PCM;
	meta->data_len_us = 1000;
	meta->sample_rate_hz = CONFIG_AUDIO_SAMPLE_RATE_HZ;
	meta->bits_per_sample = CONFIG_AUDIO_BIT_DEPTH_BITS;
	meta->carried_bits_per_sample = CONFIG_AUDIO_BIT_DEPTH_BITS;
	meta->locations = BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT;
	meta->bad_data = false;

	/* Put the block into RX queue */
	ret = k_msgq_put(audio_q_rx, (void *)&audio_block, K_NO_WAIT);
	ERR_CHK_MSG(ret, "RX failed to store block");

	if (!rx_first_data) {
		LOG_INF("USB RX first data received.");
		rx_first_data = true;
	}
}

static void uac2_buf_release_cb(const struct device *dev, uint8_t terminal, void *buf,
				void *user_data)
{

}

static void terminal_update_cb(const struct device *dev, uint8_t terminal, bool enabled,
			       bool microframes, void *user_data)
{
	terminal_enabled = enabled;
}

int audio_usb_init(void)
{
	int ret;

	const struct device *headset_dev = DEVICE_DT_GET(DT_NODELABEL(uac2_headset));
	static struct uac2_ops usb_audio_ops = {
		.sof_cb = uac2_sof_cb,
		.get_recv_buf = uac2_get_recv_buf,
		.data_recv_cb = uac2_data_recv_cb,		
		.buf_release_cb = uac2_buf_release_cb,
		.terminal_update_cb = terminal_update_cb,
	};
	struct usbd_context *sample_usbd;
	static bool initialized;
	int err;

	if (initialized) {
		return -EALREADY;
	}

	if (!device_is_ready(headset_dev)) {
		LOG_ERR("Cannot get USB Headset Device");
		return -EIO;
	}

	usbd_uac2_set_ops(headset_dev, &usb_audio_ops, NULL);

	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		return -ENODEV;
	}

	err = usbd_enable(sample_usbd);
	if (err != 0) {
		return err;
	}

	return 0;
}
