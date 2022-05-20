/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#if (CONFIG_AUDIO_DEV == HEADSET)

#include <bluetooth/bluetooth.h>
#include <bluetooth/audio/audio.h>

#include "le_audio.h"
#include "macros_common.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(bis_headset, CONFIG_LOG_BLE_LEVEL);

// TODO: Implement timeouts for the events (can use same timeout functionality for every step)

static le_audio_receive_cb receive_cb;
static bool synced_to_broadcast;

static struct bt_audio_broadcast_sink *broadcast_sink;
// TODO: Currently only 1 stream is supported
static struct bt_audio_stream streams[CONFIG_BT_AUDIO_BROADCAST_SNK_STREAM_COUNT];

/* Mandatory support preset for both source and sink */
// TODO: Change to 48_2_1 (?)
static struct bt_audio_lc3_preset preset_16_2_1 = BT_AUDIO_LC3_BROADCAST_PRESET_16_2_1;

/* Create a mask for the maximum BIS we can sync to using the number of streams
 * we have. We add an additional 1 since the bis indexes start from 1 and not
 * 0.
 */
static const uint32_t bis_index_mask = BIT_MASK(ARRAY_SIZE(streams) + 1U);
static uint32_t bis_index_bitfield;

static void stream_started_cb(struct bt_audio_stream *stream)
{
	LOG_INF("Stream started");
}

static void stream_stopped_cb(struct bt_audio_stream *stream)
{
	LOG_INF("Stream stopped");
}

static void stream_recv_cb(struct bt_audio_stream *stream, struct net_buf *buf)
{
	static uint32_t recv_cnt;

	recv_cnt++;
	if ((recv_cnt % 1000U) == 0U) {
		LOG_INF("Received %u total ISO packets", recv_cnt);
	}
}

static struct bt_audio_stream_ops stream_ops = { .started = stream_started_cb,
						 .stopped = stream_stopped_cb,
						 .recv = stream_recv_cb };

static bool scan_recv_cb(const struct bt_le_scan_recv_info *info, uint32_t broadcast_id)
{
	LOG_INF("Broadcast source found, waiting for PA sync");

	return true;
}

static void scan_term_cb(int err)
{
	if (err != 0) {
		LOG_ERR("Scan terminated with error: %d", err);
	}
}

static void pa_synced_cb(struct bt_audio_broadcast_sink *sink, struct bt_le_per_adv_sync *sync,
			 uint32_t broadcast_id)
{
	if (broadcast_sink != NULL) {
		LOG_ERR("Unexpected PA sync");
		return;
	}

	LOG_INF("PA synced for broadcast sink with broadcast ID 0x%06X", broadcast_id);

	broadcast_sink = sink;

	LOG_INF("Broadcast source PA synced, waiting for BASE");
}

static void pa_sync_lost_cb(struct bt_audio_broadcast_sink *sink)
{
	if (broadcast_sink == NULL) {
		LOG_ERR("Unexpected PA sync lost");
		return;
	}

	LOG_WRN("Sink disconnected");

	broadcast_sink = NULL;

	// TODO: Must reset connetcion, this is not implemented yet
}

static void base_recv_cb(struct bt_audio_broadcast_sink *sink, const struct bt_audio_base *base)
{
	uint32_t base_bis_index_bitfield = 0U;

	if (synced_to_broadcast) {
		return;
	}

	LOG_INF("Received BASE with %u subgroups from broadcast sink", base->subgroup_count);

	for (size_t i = 0U; i < base->subgroup_count; i++) {
		for (size_t j = 0U; j < base->subgroups[i].bis_count; j++) {
			const uint8_t index = base->subgroups[i].bis_data[j].index;

			base_bis_index_bitfield |= BIT(index);
		}
	}

	bis_index_bitfield = base_bis_index_bitfield & bis_index_mask;

	LOG_INF("BASE received, waiting for syncable");
}

static void syncable_cb(struct bt_audio_broadcast_sink *sink, bool encrypted)
{
	int ret;

	if (synced_to_broadcast) {
		return;
	}

	if (encrypted) {
		LOG_ERR("Cannot sync to encrypted broadcast source");
		return;
	}

	LOG_INF("Syncing to broadcast");

	ret = bt_audio_broadcast_sink_sync(broadcast_sink, bis_index_bitfield, streams,
					   &preset_16_2_1.codec, NULL);
	ERR_CHK_MSG(ret, "Unable to sync to broadcast source");

	synced_to_broadcast = true;
}

static struct bt_audio_broadcast_sink_cb broadcast_sink_cbs = { .scan_recv = scan_recv_cb,
								.scan_term = scan_term_cb,
								.pa_synced = pa_synced_cb,
								.pa_sync_lost = pa_sync_lost_cb,
								.base_recv = base_recv_cb,
								.syncable = syncable_cb };

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

	receive_cb = recv_cb;

	bt_audio_broadcast_sink_register_cb(&broadcast_sink_cbs);

	for (size_t i = 0U; i < ARRAY_SIZE(streams); i++) {
		streams[i].ops = &stream_ops;
	}

	bis_index_bitfield = 0U;

	if (broadcast_sink != NULL) {
		bt_audio_broadcast_sink_delete(broadcast_sink);
		broadcast_sink = NULL;
	}

	LOG_INF("Scanning for broadcast sources");

	ret = bt_audio_broadcast_sink_scan_start(BT_LE_SCAN_ACTIVE);
	ERR_CHK_MSG(ret, "Unable to start scanning for broadcast sources");
}

void le_audio_disable(void)
{
}

#endif /* (CONFIG_AUDIO_DEV == HEADSET) */
