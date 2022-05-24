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

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/audio/audio.h>
#include <bluetooth/audio/capabilities.h>

#include <sys/byteorder.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(cis_headset, CONFIG_LOG_BLE_LEVEL);

#define CHANNEL_COUNT_1 BIT(0)
#define MAX_PAC 1

static struct bt_codec lc3_codec =
	BT_CODEC_LC3(BT_CODEC_LC3_FREQ_ANY, BT_CODEC_LC3_DURATION_10, CHANNEL_COUNT_1, 40u, 120u,
		     1u, (BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | BT_AUDIO_CONTEXT_TYPE_MEDIA),
		     BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

static struct bt_conn *default_conn;
static struct bt_audio_stream streams[MAX_PAC];
static le_audio_receive_cb receive_cb;
// TODO: Implement timeouts for the events (can use same timeout functionality for every step)

static struct bt_audio_stream *lc3_config(struct bt_conn *conn,
					  struct bt_audio_ep *ep,
					  enum bt_audio_pac_type type,
					  struct bt_audio_capability *cap,
					  struct bt_codec *codec)
{
	LOG_INF("ASE Codec Config: conn %p ep %p type %u, cap %p\n",
	       (void *)conn, (void *)ep, type, (void *)cap);

	for (size_t i = 0; i < ARRAY_SIZE(streams); i++) {
		struct bt_audio_stream *stream = &streams[i];

		if (!stream->conn) {
			LOG_INF("ASE Codec Config stream %p\n", (void *)stream);
			return stream;
		}
	}

	LOG_INF("No streams available\n");

	return NULL;
}

static int lc3_reconfig(struct bt_audio_stream *stream,
			struct bt_audio_capability *cap,
			struct bt_codec *codec)
{
	LOG_INF("ASE Codec Reconfig: stream %p cap %p\n", (void *)stream, (void *)cap);

	/* We only support one QoS at the moment, reject changes */
	return -ENOEXEC;
}

static int lc3_qos(struct bt_audio_stream *stream, struct bt_codec_qos *qos)
{
	LOG_INF("QoS: stream %p qos %p\n", (void *)stream, (void *)qos);

	return 0;
}
#include "streamctrl.h"
#include "audio_system.h"
static int lc3_enable(struct bt_audio_stream *stream,
		      struct bt_codec_data *meta,
		      size_t meta_count)
{
	LOG_INF("Enable: stream %p meta_count %u\n", (void *)stream, meta_count);
	audio_system_start();
	stream_state_set(STATE_STREAMING);
	return 0;
}

static int lc3_start(struct bt_audio_stream *stream)
{
	LOG_INF("Start: stream %p\n", (void *)stream);

	return 0;
}

static int lc3_metadata(struct bt_audio_stream *stream,
			struct bt_codec_data *meta,
			size_t meta_count)
{
	LOG_INF("Metadata: stream %p meta_count %u\n", (void *)stream, meta_count);

	return 0;
}

static int lc3_disable(struct bt_audio_stream *stream)
{
	LOG_INF("Disable: stream %p\n", (void *)stream);

	return 0;
}

static int lc3_stop(struct bt_audio_stream *stream)
{
	LOG_INF("Stop: stream %p\n", (void *)stream);

	return 0;
}

static int lc3_release(struct bt_audio_stream *stream)
{
	LOG_INF("Release: stream %p\n", (void *)stream);

	return 0;
}

static struct bt_audio_capability_ops lc3_ops = {
	.config = lc3_config,
	.reconfig = lc3_reconfig,
	.qos = lc3_qos,
	.enable = lc3_enable,
	.start = lc3_start,
	.metadata = lc3_metadata,
	.disable = lc3_disable,
	.stop = lc3_stop,
	.release = lc3_release,
};

static void stream_recv(struct bt_audio_stream *stream, struct net_buf *buf)
{
	//LOG_INF("Incoming audio on stream %p len %u\n", (void *)stream, buf->len);
	static uint32_t timestamp;
	if(buf->len == 120) {
		receive_cb(buf->data, buf->len, false, timestamp);
		timestamp += 100;
	}
	//(const uint8_t *const data, size_t size, bool bad_frame,uint32_t sdu_ref);
}

static struct bt_audio_stream_ops stream_ops = {
	.recv = stream_recv
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err != 0) {


		default_conn = NULL;
		return;
	}

	LOG_INF("Connected: %s\n", addr);
	default_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static struct bt_audio_capability caps[] = {
	{
		.type = BT_AUDIO_SINK,
		.pref = BT_AUDIO_CAPABILITY_PREF(
				BT_AUDIO_CAPABILITY_UNFRAMED_SUPPORTED,
				BT_GAP_LE_PHY_2M, 0x02, 10, 10000, 10000,
				10000, 10000),
		.codec = &lc3_codec,
		.ops = &lc3_ops,
	}
};
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};


static le_audio_receive_cb receive_cb;


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
	int err;
	for (size_t i = 0; i < ARRAY_SIZE(caps); i++) {
		bt_audio_capability_register(&caps[i]);
	}

	for (size_t i = 0; i < ARRAY_SIZE(streams); i++) {
		bt_audio_stream_cb_register(&streams[i], &stream_ops);
	}
	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_INF("Advertising failed to start (err %d)\n", err);
		return;
	}
	receive_cb = recv_cb;
	bt_audio_capability_set_location(BT_AUDIO_SOURCE, BT_AUDIO_LOCATION_SIDE_LEFT);
	bt_audio_capability_set_location(BT_AUDIO_SINK, BT_AUDIO_LOCATION_SIDE_LEFT);
	LOG_INF("Advertising successfully started\n");
}

void le_audio_disable(void)
{
}

#endif /* (CONFIG_AUDIO_DEV == HEADSET) */