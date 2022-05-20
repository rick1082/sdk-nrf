/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#if (CONFIG_AUDIO_DEV == GATEWAY)

#include <bluetooth/bluetooth.h>
#include <bluetooth/audio/audio.h>

#include "le_audio.h"
#include "macros_common.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(bis_gateway, CONFIG_LOG_BLE_LEVEL);

/* When BROADCAST_ENQUEUE_COUNT > 1 we can enqueue enough buffers to ensure that
 * the controller is never idle
 */
#define BROADCAST_ENQUEUE_COUNT 2U
#define TOTAL_BUF_NEEDED (BROADCAST_ENQUEUE_COUNT * CONFIG_BT_AUDIO_BROADCAST_SRC_STREAM_COUNT)

BUILD_ASSERT(CONFIG_BT_ISO_TX_BUF_COUNT >= TOTAL_BUF_NEEDED,
	     "CONFIG_BT_ISO_TX_BUF_COUNT should be at least "
	     "BROADCAST_ENQUEUE_COUNT * CONFIG_BT_AUDIO_BROADCAST_SRC_STREAM_COUNT");

static struct bt_audio_lc3_preset preset_16_2_1 = BT_AUDIO_LC3_BROADCAST_PRESET_16_2_1;
// TODO: Currently only 1 stream is supported
static struct bt_audio_stream streams[CONFIG_BT_AUDIO_BROADCAST_SRC_STREAM_COUNT];
static struct bt_audio_broadcast_source *broadcast_source;

NET_BUF_POOL_FIXED_DEFINE(tx_pool, TOTAL_BUF_NEEDED, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8,
			  NULL);

static uint8_t mock_data[CONFIG_BT_ISO_TX_MTU];
static bool stopping;

static void stream_sent_cb(struct bt_audio_stream *stream)
{
	static uint32_t sent_cnt;
	struct net_buf *buf;
	int ret;

	if (stopping) {
		return;
	}

	buf = net_buf_alloc(&tx_pool, K_FOREVER);
	if (buf == NULL) {
		LOG_WRN("Could not allocate buffer when sending");
		return;
	}

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
	net_buf_add_mem(buf, mock_data, preset_16_2_1.qos.sdu);
	ret = bt_audio_stream_send(stream, buf);
	if (ret < 0) {
		/* This will end broadcasting on this stream. */
		LOG_WRN("Unable to broadcast data: %d", ret);
		net_buf_unref(buf);
		return;
	}

	sent_cnt++;
	if ((sent_cnt % 1000U) == 0U) {
		LOG_INF("Sent %u total ISO packets", sent_cnt);
	}
}

static void stream_started_cb(struct bt_audio_stream *stream)
{
	LOG_INF("Broadcast source started");

	/* Initialize sending */
	for (size_t i = 0U; i < ARRAY_SIZE(streams); i++) {
		for (unsigned int j = 0U; j < BROADCAST_ENQUEUE_COUNT; j++) {
			stream_sent_cb(&streams[i]);
		}
	}
}

static void stream_stopped_cb(struct bt_audio_stream *stream)
{
	LOG_INF("Broadcast source stopped");
}

static struct bt_audio_stream_ops stream_ops = { .sent = stream_sent_cb,
						 .started = stream_started_cb,
						 .stopped = stream_stopped_cb };

int le_audio_config_get(void)
{
	return 0;
}

int le_audio_volume_up(void)
{
	return 0;
}

int le_audio_volume_down(void)
{
	return 0;
}

int le_audio_volume_mute(void)
{
	return 0;
}

int le_audio_play(void)
{
	return 0;
}

int le_audio_pause(void)
{
	return 0;
}

int le_audio_send(uint8_t const *const data, size_t size)
{
	return 0;
}

void le_audio_enable(le_audio_receive_cb recv_cb)
{
	int ret;

	for (size_t i = 0U; i < ARRAY_SIZE(streams); i++) {
		streams[i].ops = &stream_ops;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(mock_data); i++) {
		/* Initialize mock data */
		mock_data[i] = i;
	}

	LOG_INF("Creating broadcast source");

	ret = bt_audio_broadcast_source_create(streams, ARRAY_SIZE(streams), &preset_16_2_1.codec,
					       &preset_16_2_1.qos, &broadcast_source);
	ERR_CHK_MSG(ret, "Unable to create broadcast source");

	LOG_INF("Starting broadcast source");

	stopping = false;

	ret = bt_audio_broadcast_source_start(broadcast_source);
	ERR_CHK_MSG(ret, "Unable to start broadcast source");
}

void le_audio_disable(void)
{
	// TODO: Stopping functionality is in broadcast_audio_source sample
}

#endif /* (CONFIG_AUDIO_DEV == GATEWAY) */
