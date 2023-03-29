/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "le_audio.h"
#include "led.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/pacs.h>
/* TODO: Remove when a get_info function is implemented in host */
#include <../subsys/bluetooth/audio/endpoint.h>
#include <../subsys/bluetooth/audio/audio_iso.h>

#include "macros_common.h"
#include "ctrl_events.h"
#include "audio_datapath.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bis_gateway, CONFIG_BLE_LOG_LEVEL);

BUILD_ASSERT(CONFIG_BT_AUDIO_BROADCAST_SRC_STREAM_COUNT <= 2,
	     "A maximum of two audio streams are currently supported");

#define HCI_ISO_BUF_ALLOC_PER_CHAN 2
#define STANDARD_QUALITY_16KHZ 16000
#define STANDARD_QUALITY_24KHZ 24000
#define HIGH_QUALITY_48KHZ 48000

/* For being able to dynamically define iso_tx_pools */
#define NET_BUF_POOL_ITERATE(i, _)                                                                 \
	NET_BUF_POOL_FIXED_DEFINE(iso_tx_pool_##i, HCI_ISO_BUF_ALLOC_PER_CHAN,                     \
				  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8, NULL);
#define NET_BUF_POOL_PTR_ITERATE(i, ...) IDENTITY(&iso_tx_pool_##i)
LISTIFY(CONFIG_BT_AUDIO_BROADCAST_SRC_STREAM_COUNT, NET_BUF_POOL_ITERATE, (;))

/* clang-format off */
static struct net_buf_pool *iso_tx_pools[] = { LISTIFY(CONFIG_BT_AUDIO_BROADCAST_SRC_STREAM_COUNT,
						       NET_BUF_POOL_PTR_ITERATE, (,)) };
/* clang-format on */

static struct bt_audio_broadcast_source *broadcast_source;

static struct bt_audio_stream audio_streams[CONFIG_BT_AUDIO_BROADCAST_SRC_STREAM_COUNT];

static struct bt_audio_lc3_preset lc3_preset = BT_AUDIO_LC3_BROADCAST_PRESET_NRF5340_AUDIO;
static int bis_headset_cleanup(bool from_sync_lost_cb);
#define BT_LE_SCAN_PASSIVE_CONTINOUS BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_PASSIVE, \
					    BT_LE_SCAN_OPT_FILTER_DUPLICATE, \
					    0x60, \
					    0x60)
struct audio_codec_info {
	uint8_t id;
	uint16_t cid;
	uint16_t vid;
	int frequency;
	int frame_duration_us;
	int chan_allocation;
	int octets_per_sdu;
	int bitrate;
	int blocks_per_sdu;
};

struct active_audio_stream {
	struct bt_audio_stream *stream;
	struct audio_codec_info *codec;
	uint8_t brdcast_src_name_idx;
};

struct bt_name {
	char *name;
	size_t size;
};

static const char *const brdcast_src_names[] = { CONFIG_BT_AUDIO_BROADCAST_NAME,
						 CONFIG_BT_AUDIO_BROADCAST_NAME_ALT };

static struct bt_audio_broadcast_sink *broadcast_sink;

static struct bt_audio_stream audio_streams_sink[CONFIG_BT_AUDIO_BROADCAST_SNK_STREAM_COUNT];
static struct bt_audio_stream *audio_streams_sink_p[ARRAY_SIZE(audio_streams_sink)];
static struct audio_codec_info audio_codec_info[CONFIG_BT_AUDIO_BROADCAST_SNK_STREAM_COUNT];
static uint32_t bis_index_bitfields[CONFIG_BT_AUDIO_BROADCAST_SNK_STREAM_COUNT];
static struct active_audio_stream active_stream;

/* The values of sync_stream_cnt and active_stream_index must never become larger
 * than the sizes of the arrays above (audio_streams_sink etc.)
 */
static uint8_t sync_stream_cnt;
static uint8_t active_stream_index;

/* We need to set a location as a pre-compile, this changed in initialize */
static struct bt_codec codec_capabilities =
	BT_CODEC_LC3_CONFIG_48_4(BT_AUDIO_LOCATION_FRONT_LEFT, BT_AUDIO_CONTEXT_TYPE_MEDIA);

static le_audio_receive_cb receive_cb;
static bool init_routine_completed;


static atomic_t iso_tx_pool_alloc[CONFIG_BT_AUDIO_BROADCAST_SRC_STREAM_COUNT];
static bool delete_broadcast_src;
static uint32_t seq_num[CONFIG_BT_AUDIO_BROADCAST_SRC_STREAM_COUNT];

static struct bt_le_ext_adv *adv;
static bool adv_data_parse(struct bt_data *data, void *user_data);

static void print_codec(const struct audio_codec_info *codec)
{
	if (codec->id == BT_CODEC_LC3_ID) {
		LOG_INF("Codec config for LC3:");
		LOG_INF("\tFrequency: %d Hz", codec->frequency);
		LOG_INF("\tFrame Duration: %d us", codec->frame_duration_us);
		LOG_INF("\tOctets per frame: %d (%d kbps)", codec->octets_per_sdu, codec->bitrate);
		LOG_INF("\tFrames per SDU: %d", codec->blocks_per_sdu);
		if (codec->chan_allocation >= 0) {
			LOG_INF("\tChannel allocation: 0x%x", codec->chan_allocation);
		}
	} else {
		LOG_WRN("Codec is not LC3, codec_id: 0x%2hhx", codec->id);
	}
}

static void get_codec_info(const struct bt_codec *codec, struct audio_codec_info *codec_info)
{
	if (codec->id == BT_CODEC_LC3_ID) {
		/* LC3 uses the generic LTV format - other codecs might do as well */
		LOG_DBG("Retrieve the codec configuration for LC3");
		codec_info->id = codec->id;
		codec_info->cid = codec->cid;
		codec_info->vid = codec->vid;
		codec_info->frequency = bt_codec_cfg_get_freq(codec);
		codec_info->frame_duration_us = bt_codec_cfg_get_frame_duration_us(codec);
		bt_codec_cfg_get_chan_allocation_val(codec, &codec_info->chan_allocation);
		codec_info->octets_per_sdu = bt_codec_cfg_get_octets_per_frame(codec);
		codec_info->bitrate =
			(codec_info->octets_per_sdu * 8 * 1000000) / codec_info->frame_duration_us;
		codec_info->blocks_per_sdu = bt_codec_cfg_get_frame_blocks_per_sdu(codec, true);
	} else {
		LOG_WRN("Codec is not LC3, codec_id: 0x%2hhx", codec->id);
	}
}

static bool bitrate_check(const struct bt_codec *codec)
{
	uint32_t octets_per_sdu = bt_codec_cfg_get_octets_per_frame(codec);

	if (octets_per_sdu < LE_AUDIO_SDU_SIZE_OCTETS(CONFIG_LC3_BITRATE_MIN)) {
		LOG_WRN("Bitrate too low");
		return false;
	} else if (octets_per_sdu > LE_AUDIO_SDU_SIZE_OCTETS(CONFIG_LC3_BITRATE_MAX)) {
		LOG_WRN("Bitrate too high");
		return false;
	}

	return true;
}

static bool scan_recv_cb(const struct bt_le_scan_recv_info *info, struct net_buf_simple *ad,
			 uint32_t broadcast_id)
{
	char name[MAX(sizeof(CONFIG_BT_AUDIO_BROADCAST_NAME),
		      sizeof(CONFIG_BT_AUDIO_BROADCAST_NAME_ALT))] = { '\0' };
	struct bt_name bis_name = { &name[0], ARRAY_SIZE(name) };

	bt_data_parse(ad, adv_data_parse, (void *)&bis_name);

	if (strlen(bis_name.name) ==
	    strlen(brdcast_src_names[active_stream.brdcast_src_name_idx])) {
		if (strncmp(bis_name.name, brdcast_src_names[active_stream.brdcast_src_name_idx],
			    strlen(brdcast_src_names[active_stream.brdcast_src_name_idx])) == 0) {
			LOG_INF("Broadcast source %s found", bis_name.name);
			return true;
		}
	}

	return false;
}

static void scan_term_cb(int err)
{
	if (err) {
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

	LOG_DBG("PA synced for broadcast sink with broadcast ID 0x%06X", broadcast_id);

	broadcast_sink = sink;

	LOG_DBG("Broadcast source PA synced, waiting for BASE");
}

static void pa_sync_lost_cb(struct bt_audio_broadcast_sink *sink)
{
	int ret;

	LOG_DBG("Periodic advertising sync lost");

	if (broadcast_sink == NULL) {
		LOG_ERR("Unexpected PA sync lost");
		return;
	}

	LOG_DBG("Sink disconnected");

	ret = bis_headset_cleanup(true);
	if (ret) {
		LOG_ERR("Error cleaning up");
		return;
	}

	LOG_INF("Restarting scanning for broadcast sources after sync lost");

	ret = bt_audio_broadcast_sink_scan_start(BT_LE_SCAN_PASSIVE_CONTINOUS);
	if (ret) {
		LOG_ERR("Unable to start scanning for broadcast sources");
	}
}

static void base_recv_cb(struct bt_audio_broadcast_sink *sink, const struct bt_audio_base *base)
{
	int ret;
	bool suitable_stream_found = false;

	if (init_routine_completed) {
		return;
	}

	LOG_DBG("Received BASE with %zu subgroup(s) from broadcast sink", base->subgroup_count);

	sync_stream_cnt = 0;

	/* Search each subgroup for the BIS of interest */
	for (int i = 0; i < base->subgroup_count; i++) {
		for (int j = 0; j < base->subgroups[i].bis_count; j++) {
			const uint8_t index = base->subgroups[i].bis_data[j].index;

			LOG_DBG("BIS %d   index = %hhu", j, index);

			if (bitrate_check((struct bt_codec *)&base->subgroups[i].codec)) {
				suitable_stream_found = true;

				bis_index_bitfields[sync_stream_cnt] = BIT(index);

				audio_streams_sink[sync_stream_cnt].codec =
					(struct bt_codec *)&base->subgroups[i].codec;
				get_codec_info(audio_streams_sink[sync_stream_cnt].codec,
					       &audio_codec_info[sync_stream_cnt]);
				print_codec(&audio_codec_info[sync_stream_cnt]);

				LOG_DBG("Stream %d in subgroup %d from broadcast sink",
					sync_stream_cnt, i);

				sync_stream_cnt += 1;
				if (sync_stream_cnt >= ARRAY_SIZE(audio_streams_sink)) {
					break;
				}
			}
		}

		if (sync_stream_cnt >= ARRAY_SIZE(audio_streams_sink)) {
			break;
		}
	}

	/* Set the initial active stream based on the defined channel of the device */
	channel_assignment_get((enum audio_channel *)&active_stream_index);
	active_stream.stream = &audio_streams_sink[active_stream_index];
	active_stream.codec = &audio_codec_info[active_stream_index];

	if (suitable_stream_found) {
		ret = ctrl_events_le_audio_event_send(LE_AUDIO_EVT_CONFIG_RECEIVED);
		ERR_CHK(ret);

		LOG_DBG("Channel %s active",
			((active_stream_index == AUDIO_CH_L) ? CH_L_TAG : CH_R_TAG));
		LOG_DBG("Waiting for syncable");
	} else {
		LOG_WRN("Found no suitable stream");
	}
}

static void syncable_cb(struct bt_audio_broadcast_sink *sink, bool encrypted)
{
	int ret;
	static uint8_t bis_encryption_key[16];

	LOG_DBG("Broadcast sink is syncable");

#if (CONFIG_BT_AUDIO_BROADCAST_ENCRYPTED)
	strncpy(bis_encryption_key, CONFIG_BT_AUDIO_BROADCAST_ENCRYPTION_KEY, 16);
#endif /* (CONFIG_BT_AUDIO_BROADCAST_ENCRYPTED) */

	/*
	if (active_stream.stream->ep->status.state == BT_AUDIO_EP_STATE_STREAMING ||
	    !playing_state) {
		LOG_DBG("Syncable received, but either in paused_state or already in a stream");
		return;
	}
	*/
	LOG_INF("Syncing to broadcast stream index %d", active_stream_index);

	ret = bt_audio_broadcast_sink_sync(broadcast_sink, bis_index_bitfields[active_stream_index],
					   audio_streams_sink_p, bis_encryption_key);
	if (ret) {
		LOG_WRN("Unable to sync to broadcast source, ret: %d", ret);
		return;
	}

	init_routine_completed = true;
}

static struct bt_audio_broadcast_sink_cb broadcast_sink_cbs = { .scan_recv = scan_recv_cb,
								.scan_term = scan_term_cb,
								.pa_synced = pa_synced_cb,
								.pa_sync_lost = pa_sync_lost_cb,
								.base_recv = base_recv_cb,
								.syncable = syncable_cb };

static struct bt_pacs_cap capabilities = {
	.codec = &codec_capabilities,
};

static int bis_headset_cleanup(bool from_sync_lost_cb)
{
	int ret;

	LOG_WRN("%s", __func__);
	ret = bt_audio_broadcast_sink_scan_stop();
	if (ret && ret != -EALREADY) {
		return ret;
	}

	if (broadcast_sink != NULL) {
		if (!from_sync_lost_cb) {
			ret = bt_audio_broadcast_sink_stop(broadcast_sink);
			if (ret && ret != -EALREADY) {
				return ret;
			}
		}

		ret = bt_audio_broadcast_sink_delete(broadcast_sink);
		if (ret && ret != -EALREADY) {
			return ret;
		}

		broadcast_sink = NULL;
	}

	init_routine_completed = false;

	return 0;
}

static bool adv_data_parse(struct bt_data *data, void *user_data)
{
	struct bt_name *bis_name = (struct bt_name *)user_data;

	if (data->type == BT_DATA_BROADCAST_NAME && data->data_len) {
		if (data->data_len <= bis_name->size) {
			memcpy(bis_name->name, data->data, data->data_len);
			return false;
		}
	}

	return true;
}


static bool is_iso_buffer_full(uint8_t idx)
{
	/* net_buf_alloc allocates buffers for APP->NET transfer over HCI RPMsg,
	 * but when these buffers are released it is not guaranteed that the
	 * data has actually been sent. The data might be qued on the NET core,
	 * and this can cause delays in the audio.
	 * When stream_sent_cb() is called the data has been sent.
	 * Data will be discarded if allocation becomes too high, to avoid audio delays.
	 * If the NET and APP core operates in clock sync, discarding should not occur.
	 */

	if (atomic_get(&iso_tx_pool_alloc[idx]) >= HCI_ISO_BUF_ALLOC_PER_CHAN) {
		return true;
	}

	return false;
}

static int get_stream_index(struct bt_audio_stream *stream, uint8_t *index)
{
	for (int i = 0; i < ARRAY_SIZE(audio_streams); i++) {
		if (&audio_streams[i] == stream) {
			*index = i;
			return 0;
		}
	}

	LOG_WRN("Stream %p not found", (void *)stream);

	return -EINVAL;
}

static void stream_sent_cb(struct bt_audio_stream *stream)
{
	static uint32_t sent_cnt[ARRAY_SIZE(audio_streams)];
	uint8_t index = 0;

	get_stream_index(stream, &index);

	if (atomic_get(&iso_tx_pool_alloc[index])) {
		atomic_dec(&iso_tx_pool_alloc[index]);
	} else {
		LOG_WRN("Decreasing atomic variable for stream %d failed", index);
	}

	sent_cnt[index]++;

	if ((sent_cnt[index] % 1000U) == 0U) {
		LOG_DBG("Sent %d total ISO packets on stream %d", sent_cnt[index], index);
	}
}

static void stream_started_cb(struct bt_audio_stream *stream)
{
	int ret;
	uint8_t index = 0;

	get_stream_index(stream, &index);
	seq_num[index] = 0;

	ret = ctrl_events_le_audio_event_send(LE_AUDIO_EVT_STREAMING);
	ERR_CHK(ret);

	LOG_INF("Broadcast source %p started", (void *)stream);
}

static void stream_stopped_cb(struct bt_audio_stream *stream)
{
	int ret;

	ret = ctrl_events_le_audio_event_send(LE_AUDIO_EVT_NOT_STREAMING);
	ERR_CHK(ret);

	LOG_INF("Broadcast source %p stopped", (void *)stream);

	if (delete_broadcast_src && broadcast_source != NULL) {
		ret = bt_audio_broadcast_source_delete(broadcast_source);
		if (ret) {
			LOG_ERR("Unable to delete broadcast source %p", (void *)stream);
			delete_broadcast_src = false;
			return;
		}

		broadcast_source = NULL;

		LOG_INF("Broadcast source %p deleted", (void *)stream);

		delete_broadcast_src = false;
	}
}

static void stream_recv_cb(struct bt_audio_stream *stream, const struct bt_iso_recv_info *info,
			   struct net_buf *buf)
{
	static uint32_t recv_cnt;
	bool bad_frame = false;

	if (receive_cb == NULL) {
		LOG_ERR("The RX callback has not been set");
		return;
	}

	if (!(info->flags & BT_ISO_FLAGS_VALID)) {
		bad_frame = true;
	}

	receive_cb(buf->data, buf->len, bad_frame, info->ts, active_stream_index);

	recv_cnt++;
	if ((recv_cnt % 1000U) == 0U) {
		LOG_DBG("Received %d total ISO packets for stream %d", recv_cnt,
			active_stream_index);
	}
}

static struct bt_audio_stream_ops stream_ops = { .sent = stream_sent_cb,
						 .started = stream_started_cb,
						 .stopped = stream_stopped_cb,
						 .recv = stream_recv_cb };

/** Non-connectable extended advertising with @ref BT_LE_ADV_OPT_USE_NAME */
#define BT_LE_EXT_ADV_NCONN_NAME_FAST BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV | \
						 BT_LE_ADV_OPT_USE_NAME, \
						 BT_GAP_ADV_FAST_INT_MIN_1, \
						 BT_GAP_ADV_FAST_INT_MAX_1, \
						 NULL)

#define BT_LE_PER_ADV_FAST BT_LE_PER_ADV_PARAM(0x0018, \
						  0x0030, \
						  BT_LE_PER_ADV_OPT_NONE)
static int adv_create(void)
{
	int ret;

	/* Broadcast Audio Streaming Endpoint advertising data */
	NET_BUF_SIMPLE_DEFINE(ad_buf, BT_UUID_SIZE_16 + BT_AUDIO_BROADCAST_ID_SIZE);
	/* Buffer for Public Broadcast Announcement */
	NET_BUF_SIMPLE_DEFINE(base_buf, 128);

	struct bt_data ext_ad[3];

	struct bt_data per_ad;

	uint32_t broadcast_id = 0;

	/* Create a non-connectable non-scannable advertising set */
	ret = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN_NAME_FAST, NULL, &adv);
	if (ret) {
		LOG_ERR("Unable to create extended advertising set: %d", ret);
		return ret;
	}

	/* Set periodic advertising parameters */
	ret = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_FAST);
	if (ret) {
		LOG_ERR("Failed to set periodic advertising parameters (ret %d)", ret);
		return ret;
	}

	if (IS_ENABLED(CONFIG_BT_AUDIO_USE_BROADCAST_ID_RANDOM)) {
		ret = bt_audio_broadcast_source_get_id(broadcast_source, &broadcast_id);
		if (ret) {
			LOG_ERR("Unable to get broadcast ID: %d", ret);
			return ret;
		}
	} else {
		broadcast_id = CONFIG_BT_AUDIO_BROADCAST_ID_FIXED;
	}

	ext_ad[0] = (struct bt_data)BT_DATA_BYTES(BT_DATA_BROADCAST_NAME,
						  CONFIG_BT_AUDIO_BROADCAST_NAME);

	/* Setup extended advertising data */
	net_buf_simple_add_le16(&ad_buf, BT_UUID_BROADCAST_AUDIO_VAL);
	net_buf_simple_add_le24(&ad_buf, broadcast_id);

	ext_ad[1] = (struct bt_data)BT_DATA(BT_DATA_SVC_DATA16, ad_buf.data, ad_buf.len);

	ext_ad[2] = (struct bt_data)BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
						  (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
						  (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff);

	ret = bt_le_ext_adv_set_data(adv, ext_ad, ARRAY_SIZE(ext_ad), NULL, 0);
	if (ret) {
		LOG_ERR("Failed to set extended advertising data: %d", ret);
		return ret;
	}

	/* Setup periodic advertising data */
	ret = bt_audio_broadcast_source_get_base(broadcast_source, &base_buf);
	if (ret) {
		LOG_ERR("Failed to get encoded BASE: %d", ret);
		return ret;
	}

	per_ad.type = BT_DATA_SVC_DATA16;
	per_ad.data_len = base_buf.len;
	per_ad.data = base_buf.data;

	ret = bt_le_per_adv_set_data(adv, &per_ad, 1);
	if (ret) {
		LOG_ERR("Failed to set periodic advertising data: %d", ret);
		return ret;
	}

	return 0;
}

static int initialize(le_audio_receive_cb recv_cb)
{
	int ret;
	static bool initialized;
	struct bt_codec_data bis_codec_data =
		BT_CODEC_DATA(BT_CODEC_CONFIG_LC3_FREQ, BT_AUDIO_CODEC_CONFIG_FREQ);
	struct bt_audio_broadcast_source_stream_param stream_params[ARRAY_SIZE(audio_streams)];
	struct bt_audio_broadcast_source_subgroup_param
		subgroup_params[CONFIG_BT_AUDIO_BROADCAST_SRC_SUBGROUP_COUNT];
	struct bt_audio_broadcast_source_create_param create_param;

	if (initialized) {
		LOG_WRN("Already initialized");
		return -EALREADY;
	}

	receive_cb = recv_cb;
	(void)memset(audio_streams, 0, sizeof(audio_streams));

	for (size_t i = 0; i < ARRAY_SIZE(stream_params); i++) {
		stream_params[i].stream = &audio_streams[i];
		bt_audio_stream_cb_register(stream_params[i].stream, &stream_ops);
		stream_params[i].data_count = 1U;
		stream_params[i].data = &bis_codec_data;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(subgroup_params); i++) {
		subgroup_params[i].params_count = ARRAY_SIZE(stream_params);
		subgroup_params[i].params = &stream_params[i];
		subgroup_params[i].codec = &lc3_preset.codec;
#if (CONFIG_BT_AUDIO_BROADCAST_IMMEDIATE_FLAG)
		/* Immediate rendering flag */
		subgroup_params[i].codec->meta[0].data.type = 0x09;
		subgroup_params[i].codec->meta[0].data.data_len = 0;
		subgroup_params[i].codec->meta_count = 1;
#endif /* (CONFIG_BT_AUDIO_BROADCAST_IMMEDIATE_FLAG) */
	}

	create_param.params_count = ARRAY_SIZE(subgroup_params);
	create_param.params = subgroup_params;
	create_param.qos = &lc3_preset.qos;

	if (IS_ENABLED(CONFIG_BT_AUDIO_PACKING_INTERLEAVED)) {
		create_param.packing = BT_ISO_PACKING_INTERLEAVED;
	} else {
		create_param.packing = BT_ISO_PACKING_SEQUENTIAL;
	}

	if (IS_ENABLED(CONFIG_BT_AUDIO_BROADCAST_ENCRYPTED)) {
		create_param.encryption = true;
		memset(create_param.broadcast_code, 0, sizeof(create_param.broadcast_code));
		memcpy(create_param.broadcast_code, CONFIG_BT_AUDIO_BROADCAST_ENCRYPTION_KEY,
		       MIN(sizeof(CONFIG_BT_AUDIO_BROADCAST_ENCRYPTION_KEY),
			   sizeof(create_param.broadcast_code)));
	} else {
		create_param.encryption = false;
	}

	LOG_DBG("Creating broadcast source");

	ret = bt_audio_broadcast_source_create(&create_param, &broadcast_source);

	if (ret) {
		LOG_ERR("Failed to create broadcast source, ret: %d", ret);
		return ret;
	}

	/* Create advertising set */
	ret = adv_create();

	if (ret) {
		LOG_ERR("Failed to create advertising set");
		return ret;
	}

	ret = bt_pacs_cap_register(BT_AUDIO_DIR_SINK, &capabilities);
	if (ret) {
		LOG_ERR("Capability register failed (ret %d)", ret);
		ERR_CHK(ret);
	}

	bt_audio_broadcast_sink_register_cb(&broadcast_sink_cbs);

	for (int i = 0; i < ARRAY_SIZE(audio_streams_sink); i++) {
		audio_streams_sink_p[i] = &audio_streams_sink[i];
		audio_streams_sink[i].ops = &stream_ops;
	}

	initialized = true;
	return 0;
}

int le_audio_user_defined_button_press(enum le_audio_user_defined_action action)
{
	return 0;
}

int le_audio_config_get(uint32_t *bitrate, uint32_t *sampling_rate)
{
	if (active_stream.codec == NULL) {
		return -ECANCELED;
	}

	*sampling_rate = active_stream.codec->frequency;
	*bitrate = active_stream.codec->bitrate;

	return 0;
}

int le_audio_volume_up(void)
{
	LOG_WRN("Not possible to increase volume on/from broadcast source");
	return -ENXIO;
}

int le_audio_volume_down(void)
{
	LOG_WRN("Not possible to decrease volume on/from broadcast source");
	return -ENXIO;
}

int le_audio_volume_mute(void)
{
	LOG_WRN("Not possible to mute volume on/from broadcast source");
	return -ENXIO;
}

int le_audio_play_pause(void)
{
	int ret;
	static bool play_state = true;

	if (play_state == true) {
		led_on(LED_APP_RGB, LED_COLOR_BLUE);
		LOG_WRN("Receive mode");
		ret = bt_audio_broadcast_source_stop(broadcast_source);
		if (ret) {
			LOG_WRN("Failed to stop broadcast, ret: %d", ret);
		}
		/* Enable Periodic Advertising */
		ret = bt_le_per_adv_stop(adv);
		if (ret) {
			LOG_ERR("Failed to bt_le_per_adv_stop: %d", ret);
		}

		ret = bt_le_ext_adv_stop(adv);
		if (ret) {
			LOG_ERR("Failed to bt_le_ext_adv_stop: %d", ret);
		}

		ret = bt_audio_broadcast_sink_scan_start(BT_LE_SCAN_PASSIVE_CONTINOUS);
		if (ret) {
			LOG_ERR("Failed to bt_audio_broadcast_sink_scan_start: %d", ret);
		}

		play_state = false;

	} else {
		led_on(LED_APP_RGB, LED_COLOR_GREEN);
		LOG_WRN("Transmit mode");

		ret = bt_audio_broadcast_sink_scan_stop();
		if (ret) {
			LOG_ERR("Failed to bt_audio_broadcast_sink_scan_stop: %d", ret);
		}

		ret = bt_audio_broadcast_sink_stop(broadcast_sink);
		if (ret) {
			LOG_ERR("Failed to stop broadcast sink: %d", ret);
		}
		
		ret = bis_headset_cleanup(false);
		if (ret) {
			LOG_ERR("Failed to bis_headset_cleanup: %d", ret);
		}

		ret = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
		if (ret) {
			LOG_ERR("Failed to start extended advertising: %d", ret);
		}

		/* Enable Periodic Advertising */
		ret = bt_le_per_adv_start(adv);
		if (ret) {
			LOG_ERR("Failed to enable periodic advertising: %d", ret);
		}

		ret = bt_audio_broadcast_source_start(broadcast_source, adv);
		if (ret) {
			LOG_WRN("Failed to start broadcast, ret: %d", ret);
		}

		play_state = true;
	}

	return 0;
}

int le_audio_send(struct encoded_audio enc_audio)
{
	int ret;
	static bool wrn_printed[CONFIG_BT_AUDIO_BROADCAST_SRC_STREAM_COUNT];
	struct net_buf *buf;
	size_t num_streams = ARRAY_SIZE(audio_streams);
	size_t data_size_pr_stream;

	if ((enc_audio.num_ch == 1) || (enc_audio.num_ch == num_streams)) {
		data_size_pr_stream = enc_audio.size / enc_audio.num_ch;
	} else {
		LOG_ERR("Num encoded channels must be 1 or equal to num streams");
		return -EINVAL;
	}

	if (data_size_pr_stream != LE_AUDIO_SDU_SIZE_OCTETS(CONFIG_LC3_BITRATE)) {
		LOG_ERR("The encoded data size does not match the SDU size");
		return -ECANCELED;
	}

	for (int i = 0; i < num_streams; i++) {
		if (audio_streams[i].ep->status.state != BT_AUDIO_EP_STATE_STREAMING) {
			LOG_DBG("Stream %d not in streaming state", i);
			continue;
		}

		if (is_iso_buffer_full(i)) {
			if (!wrn_printed[i]) {
				LOG_WRN("HCI ISO TX overrun on ch %d - Single print", i);
				wrn_printed[i] = true;
			}

			return -ENOMEM;
		}

		wrn_printed[i] = false;

		buf = net_buf_alloc(iso_tx_pools[i], K_NO_WAIT);
		if (buf == NULL) {
			/* This should never occur because of the is_iso_buffer_full() check */
			LOG_WRN("Out of TX buffers");
			return -ENOMEM;
		}

		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
		if (enc_audio.num_ch == 1) {
			net_buf_add_mem(buf, &enc_audio.data[0], data_size_pr_stream);
		} else {
			net_buf_add_mem(buf, &enc_audio.data[i * data_size_pr_stream],
					data_size_pr_stream);
		}

		atomic_inc(&iso_tx_pool_alloc[i]);

		ret = bt_audio_stream_send(&audio_streams[i], buf, seq_num[i]++,
					   BT_ISO_TIMESTAMP_NONE);
		if (ret < 0) {
			LOG_WRN("Failed to send audio data: %d", ret);
			net_buf_unref(buf);
			atomic_dec(&iso_tx_pool_alloc[i]);
			return ret;
		}
	}

#if (CONFIG_AUDIO_SOURCE_I2S)
	struct bt_iso_tx_info tx_info = { 0 };

	ret = bt_iso_chan_get_tx_sync(&audio_streams[0].ep->iso->chan, &tx_info);

	if (ret) {
		LOG_DBG("Error getting ISO TX anchor point: %d", ret);
	} else {
		audio_datapath_sdu_ref_update(tx_info.ts);
	}
#endif

	return 0;
}

int le_audio_enable(le_audio_receive_cb recv_cb)
{
	int ret;

	ARG_UNUSED(recv_cb);

	LOG_INF("Starting broadcast gateway %s", CONFIG_BT_AUDIO_BROADCAST_NAME);

	ret = initialize(recv_cb);
	if (ret) {
		LOG_ERR("Failed to initialize");
		return ret;
	}

	/* Start extended advertising */
	ret = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to start extended advertising: %d", ret);
		return ret;
	}

	/* Enable Periodic Advertising */
	ret = bt_le_per_adv_start(adv);
	if (ret) {
		LOG_ERR("Failed to enable periodic advertising: %d", ret);
		return ret;
	}

	LOG_DBG("Starting broadcast source");

	ret = bt_audio_broadcast_source_start(broadcast_source, adv);
	if (ret) {
		return ret;
	}

	LOG_DBG("LE Audio enabled");
	le_audio_play_pause();
	return 0;
}

int le_audio_disable(void)
{
	int ret;

	if (audio_streams[0].ep->status.state == BT_AUDIO_EP_STATE_STREAMING) {
		/* Deleting broadcast source in stream_stopped_cb() */
		delete_broadcast_src = true;

		ret = bt_audio_broadcast_source_stop(broadcast_source);
		if (ret) {
			return ret;
		}
	} else if (broadcast_source != NULL) {
		ret = bt_audio_broadcast_source_delete(broadcast_source);
		if (ret) {
			return ret;
		}

		broadcast_source = NULL;
	}

	LOG_DBG("LE Audio disabled");

	return 0;
}
