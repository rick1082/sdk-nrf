/*
 * Copyright (c) 2021-2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* --- Standard includes --- */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>

/* --- Zephyr core includes --- */
#include <zephyr/autoconf.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/sys_clock.h>
#include <zephyr/types.h>

/* --- Bluetooth includes --- */
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/lc3.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>

/* --- LC3 codec --- */
#if defined(CONFIG_LIBLC3)
#include "lc3.h"
#endif

/* --- Audio and context definitions --- */
#define AVAILABLE_SINK_CONTEXT  (BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | \
				 BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | \
				 BT_AUDIO_CONTEXT_TYPE_MEDIA | \
				 BT_AUDIO_CONTEXT_TYPE_GAME | \
				 BT_AUDIO_CONTEXT_TYPE_INSTRUCTIONAL)

#define AVAILABLE_SOURCE_CONTEXT (BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | \
				  BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | \
				  BT_AUDIO_CONTEXT_TYPE_MEDIA | \
				  BT_AUDIO_CONTEXT_TYPE_GAME)

#define AUDIO_DATA_TIMEOUT_US 1000000UL /* Send data every 1 second */
#define SDU_INTERVAL_US       10000UL   /* 10 ms SDU interval */

#define AUDIO_VOLUME            (INT16_MAX - 3000) /* codec does clipping above INT16_MAX - 3000 */
#define AUDIO_TONE_FREQUENCY_HZ 400

#if defined(CONFIG_LIBLC3)
#define MAX_SAMPLE_RATE         48000
#define MAX_FRAME_DURATION_US   10000
#define MAX_NUM_SAMPLES         ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)
#endif

/* --- Buffer pools --- */
NET_BUF_POOL_FIXED_DEFINE(tx_pool, CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

/* --- Codec capabilities --- */
static const struct bt_audio_codec_cap lc3_codec_cap = BT_AUDIO_CODEC_CAP_LC3(
	BT_AUDIO_CODEC_CAP_FREQ_ANY, BT_AUDIO_CODEC_CAP_DURATION_10,
	BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1), 40u, 120u, 1u,
	(BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | BT_AUDIO_CONTEXT_TYPE_MEDIA));

/* --- QoS preferences --- */
static const struct bt_bap_qos_cfg_pref qos_pref =
	BT_BAP_QOS_CFG_PREF(true, BT_GAP_LE_PHY_2M, 0x02, 10, 40000, 40000, 40000, 40000);

/* --- Connection and stream management --- */
static struct bt_conn *default_conn;
static struct k_work_delayable audio_send_work;
static struct bt_bap_stream sink_streams[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];

static struct audio_source {
	struct bt_bap_stream stream;
	uint16_t seq_num;
	uint16_t max_sdu;
	size_t len_to_send;
} source_streams[CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT];

static size_t configured_source_stream_count;

/* --- Synchronization --- */
static K_SEM_DEFINE(sem_disconnected, 0, 1);

/* --- LC3 codec variables --- */
#if defined(CONFIG_LIBLC3)
static lc3_encoder_t lc3_encoder;
static lc3_encoder_mem_48k_t lc3_encoder_mem;
static int frames_per_sdu;
static int octets_per_frame;
static int16_t send_pcm_data[MAX_NUM_SAMPLES];
#endif

/* --- Advertising data --- */
static uint8_t unicast_server_addata[] = {
	BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),
	BT_AUDIO_UNICAST_ANNOUNCEMENT_TARGETED,
	BT_BYTES_LIST_LE16(AVAILABLE_SINK_CONTEXT),
	BT_BYTES_LIST_LE16(AVAILABLE_SOURCE_CONTEXT),
	0x00, /* Metadata length */
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL)),
	BT_DATA(BT_DATA_SVC_DATA16, unicast_server_addata, ARRAY_SIZE(unicast_server_addata)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* --- Helper functions --- */
static uint16_t get_and_incr_seq_num(const struct bt_bap_stream *stream)
{
	for (size_t i = 0U; i < configured_source_stream_count; i++) {
		if (stream == &source_streams[i].stream) {
			uint16_t seq_num;

			seq_num = source_streams[i].seq_num;

			if (IS_ENABLED(CONFIG_LIBLC3)) {
				source_streams[i].seq_num++;
			} else {
				source_streams[i].seq_num += (AUDIO_DATA_TIMEOUT_US /
							      SDU_INTERVAL_US);
			}

			return seq_num;
		}
	}

	printk("Could not find endpoint from stream %p\n", stream);

	return 0;
}

void print_hex(const uint8_t *ptr, size_t len)
{
	while (len-- != 0) {
		printk("%02x", *ptr++);
	}
}

static bool print_cb(struct bt_data *data, void *user_data)
{
	const char *str = (const char *)user_data;

	printk("%s: type 0x%02x value_len %u\n", str, data->type, data->data_len);
	print_hex(data->data, data->data_len);
	printk("\n");

	return true;
}

static void print_codec_cfg(const struct bt_audio_codec_cfg *codec_cfg)
{
	printk("codec_cfg 0x%02x cid 0x%04x vid 0x%04x count %u\n", codec_cfg->id, codec_cfg->cid,
	       codec_cfg->vid, codec_cfg->data_len);

	if (codec_cfg->id == BT_HCI_CODING_FORMAT_LC3) {
		enum bt_audio_location chan_allocation;
		int ret;

		/* LC3 uses the generic LTV format - other codecs might do as well */

		bt_audio_data_parse(codec_cfg->data, codec_cfg->data_len, print_cb, "data");

		ret = bt_audio_codec_cfg_get_freq(codec_cfg);
		if (ret > 0) {
			printk("  Frequency: %d Hz\n", bt_audio_codec_cfg_freq_to_freq_hz(ret));
		}

		ret = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
		if (ret > 0) {
			printk("  Frame Duration: %d us\n",
			       bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret));
		}

		ret = bt_audio_codec_cfg_get_chan_allocation(codec_cfg, &chan_allocation, false);
		if (ret == 0) {
			printk("  Channel allocation: 0x%x\n", chan_allocation);
		}

		octets_per_frame = bt_audio_codec_cfg_get_octets_per_frame(codec_cfg);
		printk("  Octets per frame: %d (negative means value not pressent)\n",
		       octets_per_frame);
		
		printk("  Frames per SDU: %d\n",
		       bt_audio_codec_cfg_get_frame_blocks_per_sdu(codec_cfg, true));
	} else {
		print_hex(codec_cfg->data, codec_cfg->data_len);
	}

	bt_audio_data_parse(codec_cfg->meta, codec_cfg->meta_len, print_cb, "meta");
}

static void print_qos(const struct bt_bap_qos_cfg *qos)
{
	printk("QoS: interval %u framing 0x%02x phy 0x%02x sdu %u "
	       "rtn %u latency %u pd %u\n",
	       qos->interval, qos->framing, qos->phy, qos->sdu,
	       qos->rtn, qos->latency, qos->pd);
}

#if defined(CONFIG_LIBLC3)
static void fill_audio_buf_sin(int16_t *buf, int length_us, int frequency_hz, int sample_rate_hz)
{
	const int sine_period_samples = sample_rate_hz / frequency_hz;
	const unsigned int num_samples = (length_us * sample_rate_hz) / USEC_PER_SEC;
	const float step = 2 * 3.1415f / sine_period_samples;

	for (unsigned int i = 0; i < num_samples; i++) {
		const float sample = sinf(i * step);

		buf[i] = (int16_t)(AUDIO_VOLUME * sample);
	}
}
#endif

/* --- Audio processing --- */
static void audio_timer_timeout(struct k_work *work)
{
	int ret;
	static uint8_t buf_data[CONFIG_BT_ISO_TX_MTU];
	static bool data_initialized;
	struct net_buf *buf;

	/* We configured the sink streams to be first in `streams`, so that
	 * we can use `stream[i]` to select sink streams (i.e. streams with
	 * data going to the server)
	 */
	for (size_t i = 0; i < configured_source_stream_count; i++) {
		struct bt_bap_stream *stream = &source_streams[i].stream;

		buf = net_buf_alloc(&tx_pool, K_NO_WAIT);
		if (buf == NULL) {
			printk("Failed to allocate TX buffer\n");
			/* Break and retry later */
			break;
		}
		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
		
		ret = lc3_encode(lc3_encoder, LC3_PCM_FORMAT_S16, send_pcm_data, 1,
			 octets_per_frame, buf_data);
		net_buf_add_mem(buf, buf_data, octets_per_frame);

		ret = bt_bap_stream_send(stream, buf, get_and_incr_seq_num(stream));
		if (ret < 0) {
			printk("Failed to send audio data on streams[%zu] (%p): (%d)\n",
			       i, stream, ret);
			net_buf_unref(buf);
			k_work_schedule(&audio_send_work, K_USEC(AUDIO_DATA_TIMEOUT_US));
		}

	}
}

/* --- Stream management functions --- */
static enum bt_audio_dir stream_dir(const struct bt_bap_stream *stream)
{
	for (size_t i = 0U; i < ARRAY_SIZE(source_streams); i++) {
		if (stream == &source_streams[i].stream) {
			return BT_AUDIO_DIR_SOURCE;
		}
	}

	for (size_t i = 0U; i < ARRAY_SIZE(sink_streams); i++) {
		if (stream == &sink_streams[i]) {
			return BT_AUDIO_DIR_SINK;
		}
	}

	__ASSERT(false, "Invalid stream %p", stream);
	return 0;
}

static struct bt_bap_stream *stream_alloc(enum bt_audio_dir dir)
{
	if (dir == BT_AUDIO_DIR_SOURCE) {
		for (size_t i = 0; i < ARRAY_SIZE(source_streams); i++) {
			struct bt_bap_stream *stream = &source_streams[i].stream;

			if (!stream->conn) {
				return stream;
			}
		}
	} else {
		for (size_t i = 0; i < ARRAY_SIZE(sink_streams); i++) {
			struct bt_bap_stream *stream = &sink_streams[i];

			if (!stream->conn) {
				return stream;
			}
		}
	}

	return NULL;
}

/* --- BAP unicast server callbacks --- */
static int lc3_config(struct bt_conn *conn, const struct bt_bap_ep *ep, enum bt_audio_dir dir,
		      const struct bt_audio_codec_cfg *codec_cfg, struct bt_bap_stream **stream,
		      struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	printk("ASE Codec Config: conn %p ep %p dir %u\n", conn, ep, dir);

	print_codec_cfg(codec_cfg);

	*stream = stream_alloc(dir);
	if (*stream == NULL) {
		printk("No streams available\n");
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_NO_MEM, BT_BAP_ASCS_REASON_NONE);

		return -ENOMEM;
	}

	printk("ASE Codec Config stream %p\n", *stream);

	if (dir == BT_AUDIO_DIR_SOURCE) {
		configured_source_stream_count++;
	}

	*pref = qos_pref;

#if defined(CONFIG_LIBLC3)
	/* Nothing to free as static memory is used */
	lc3_encoder = NULL;
#endif

	return 0;
}

static int lc3_reconfig(struct bt_bap_stream *stream, enum bt_audio_dir dir,
			const struct bt_audio_codec_cfg *codec_cfg,
			struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	printk("ASE Codec Reconfig: stream %p\n", stream);

	print_codec_cfg(codec_cfg);

#if defined(CONFIG_LIBLC3)
	/* Nothing to free as static memory is used */
	lc3_encoder = NULL;
#endif

	*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_UNSUPPORTED, BT_BAP_ASCS_REASON_NONE);

	/* We only support one QoS at the moment, reject changes */
	return -ENOEXEC;
}

static int lc3_qos(struct bt_bap_stream *stream, const struct bt_bap_qos_cfg *qos,
		   struct bt_bap_ascs_rsp *rsp)
{
	printk("QoS: stream %p qos %p\n", stream, qos);

	print_qos(qos);

	for (size_t i = 0U; i < configured_source_stream_count; i++) {
		if (stream == &source_streams[i].stream) {
			source_streams[i].max_sdu = qos->sdu;
			break;
		}
	}

	return 0;
}

static int lc3_enable(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len,
		      struct bt_bap_ascs_rsp *rsp)
{
	printk("Enable: stream %p meta_len %zu\n", stream, meta_len);

	{
		int frame_duration_us;
		int freq;
		int ret;

		ret = bt_audio_codec_cfg_get_freq(stream->codec_cfg);
		if (ret > 0) {
			freq = bt_audio_codec_cfg_freq_to_freq_hz(ret);
		} else {
			printk("Error: Codec frequency not set, cannot start codec.");
			*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
					       BT_BAP_ASCS_REASON_CODEC_DATA);
			return ret;
		}

		ret = bt_audio_codec_cfg_get_frame_dur(stream->codec_cfg);
		if (ret > 0) {
			frame_duration_us = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);
		} else {
			printk("Error: Frame duration not set, cannot start codec.");
			*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
					       BT_BAP_ASCS_REASON_CODEC_DATA);
			return ret;
		}

		frames_per_sdu =
			bt_audio_codec_cfg_get_frame_blocks_per_sdu(stream->codec_cfg, true);
		
		lc3_encoder = lc3_setup_encoder(frame_duration_us,
					       freq,
					       0,
					       &lc3_encoder_mem);
		fill_audio_buf_sin(send_pcm_data, frame_duration_us, AUDIO_TONE_FREQUENCY_HZ, freq);
		if (lc3_encoder == NULL) {
			printk("ERROR: Failed to setup LC3 encoder - wrong parameters?\n");
			*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
					       BT_BAP_ASCS_REASON_CODEC_DATA);
			return -1;
		}
	}

	return 0;
}

static int lc3_start(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	printk("Start: stream %p\n", stream);

	for (size_t i = 0U; i < configured_source_stream_count; i++) {
		if (stream == &source_streams[i].stream) {
			source_streams[i].seq_num = 0U;
			source_streams[i].len_to_send = 0U;
			break;
		}
	}

	if (configured_source_stream_count > 0 &&
	    !k_work_delayable_is_pending(&audio_send_work)) {

		/* Start send timer */
		k_work_schedule(&audio_send_work, K_MSEC(0));
	}

	return 0;
}

static bool data_func_cb(struct bt_data *data, void *user_data)
{
	struct bt_bap_ascs_rsp *rsp = (struct bt_bap_ascs_rsp *)user_data;

	if (!BT_AUDIO_METADATA_TYPE_IS_KNOWN(data->type)) {
		printk("Invalid metadata type %u or length %u\n", data->type, data->data_len);
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_METADATA_REJECTED, data->type);

		return -EINVAL;
	}

	return true;
}

static int lc3_metadata(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len,
			struct bt_bap_ascs_rsp *rsp)
{
	printk("Metadata: stream %p meta_len %zu\n", stream, meta_len);

	return bt_audio_data_parse(meta, meta_len, data_func_cb, rsp);
}

static int lc3_disable(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	printk("Disable: stream %p\n", stream);

	return 0;
}

static int lc3_stop(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	printk("Stop: stream %p\n", stream);

	return 0;
}

static int lc3_release(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	printk("Release: stream %p\n", stream);
	return 0;
}

static struct bt_bap_unicast_server_register_param param = {
	CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT,
	CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT
};

static const struct bt_bap_unicast_server_cb unicast_server_cb = {
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

/* --- Stream operations --- */
static void stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
	printk("Audio Stream %p stopped with reason 0x%02X\n", stream, reason);

	/* Stop send timer */
	k_work_cancel_delayable(&audio_send_work);
}

static void stream_started(struct bt_bap_stream *stream)
{
	printk("Audio Stream %p started\n", stream);
}

static void stream_sent(struct bt_bap_stream *stream)
{
	k_work_schedule(&audio_send_work, K_USEC(0));
}

static void stream_enabled_cb(struct bt_bap_stream *stream)
{
	/* The unicast server is responsible for starting sink ASEs after the
	 * client has enabled them.
	 */
	if (stream_dir(stream) == BT_AUDIO_DIR_SINK) {
		const int err = bt_bap_stream_start(stream);

		if (err != 0) {
			printk("Failed to start stream %p: %d", stream, err);
		}
	}
}

static struct bt_bap_stream_ops stream_ops = {
	.recv = stream_recv,
	.stopped = stream_stopped,
	.started = stream_started,
	.enabled = stream_enabled_cb,
	.sent = stream_sent,
};

/* --- Connection callbacks --- */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err != 0) {
		printk("Failed to connect to %s %u %s\n", addr, err, bt_hci_err_to_str(err));

		default_conn = NULL;
		return;
	}

	printk("Connected: %s\n", addr);
	default_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

	bt_conn_unref(default_conn);
	default_conn = NULL;

	k_sem_give(&sem_disconnected);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/* --- PACS capabilities --- */
static struct bt_pacs_cap cap_source = {
	.codec_cap = &lc3_codec_cap,
};

/* --- PACS setup functions --- */
static int set_location(void)
{
	int err;

	if (IS_ENABLED(CONFIG_BT_PAC_SRC_LOC)) {
		err = bt_pacs_set_location(BT_AUDIO_DIR_SOURCE,
					   (BT_AUDIO_LOCATION_FRONT_LEFT));
		if (err != 0) {
			printk("Failed to set source location (err %d)\n", err);
			return err;
		}
	}

	printk("Location successfully set\n");

	return 0;
}

static int set_supported_contexts(void)
{
	int err;

	if (IS_ENABLED(CONFIG_BT_PAC_SRC)) {
		err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SOURCE,
						     AVAILABLE_SOURCE_CONTEXT);
		if (err != 0) {
			printk("Failed to set source supported contexts (err %d)\n",
			       err);

			return err;
		}
	}

	printk("Supported contexts successfully set\n");

	return 0;
}

static int set_available_contexts(void)
{
	int err;

	if (IS_ENABLED(CONFIG_BT_PAC_SRC)) {
		err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SOURCE,
						     AVAILABLE_SOURCE_CONTEXT);
		if (err != 0) {
			printk("Failed to set source available contexts (err %d)\n", err);
			return err;
		}
	}

	printk("Available contexts successfully set\n");
	return 0;
}

/* --- Main function --- */
int main(void)
{
	struct bt_le_ext_adv *adv;
	const struct bt_pacs_register_param pacs_param = {
		.snk_pac = false,
		.snk_loc = false,
		.src_pac = true,
		.src_loc = true,
	};
	int err;

	/* Bluetooth initialization */
	err = bt_enable(NULL);
	if (err != 0) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	printk("Bluetooth initialized\n");

	/* PACS registration */
	err = bt_pacs_register(&pacs_param);
	if (err) {
		printk("Could not register PACS (err %d)\n", err);
		return 0;
	}

	/* BAP server setup */
	bt_bap_unicast_server_register(&param);
	bt_bap_unicast_server_register_cb(&unicast_server_cb);

	bt_pacs_cap_register(BT_AUDIO_DIR_SOURCE, &cap_source);

	/* Stream callback registration */
	for (size_t i = 0; i < ARRAY_SIZE(source_streams); i++) {
		bt_bap_stream_cb_register(&source_streams[i].stream, &stream_ops);
	}

	/* PACS configuration */
	err = set_location();
	if (err != 0) {
		return 0;
	}

	err = set_supported_contexts();
	if (err != 0) {
		return 0;
	}

	err = set_available_contexts();
	if (err != 0) {
		return 0;
	}

	/* Advertising setup */
	err = bt_le_ext_adv_create(BT_BAP_ADV_PARAM_CONN_QUICK, NULL, &adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return 0;
	}

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set advertising data (err %d)\n", err);
		return 0;
	}

	/* Main loop */
	while (true) {
		struct k_work_sync sync;

		err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
		if (err) {
			printk("Failed to start advertising set (err %d)\n", err);
			return 0;
		}

		printk("Advertising successfully started\n");

		if (CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT > 0) {
			/* Initialize audio send timer */
			k_work_init_delayable(&audio_send_work, audio_timer_timeout);
		}

		err = k_sem_take(&sem_disconnected, K_FOREVER);
		if (err != 0) {
			printk("failed to take sem_disconnected (err %d)\n", err);
			return 0;
		}

		/* Reset data after disconnection */
		configured_source_stream_count = 0U;
		k_work_cancel_delayable_sync(&audio_send_work, &sync);
	}

	return 0;
}
