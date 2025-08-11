/*
 * Copyright (c) 2021-2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <dk_buttons_and_leds.h>
#include "sw_codec_lc3.h"
#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/bluetooth/services/bas.h>
#include <bluetooth/services/hids.h>
#include <zephyr/bluetooth/services/dis.h>
#include <zephyr/settings/settings.h>

#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

#define BASE_USB_HID_SPEC_VERSION 0x0101

/* Number of pixels by which the cursor is moved when a button is pushed. */
#define MOVEMENT_SPEED		   10
/* Number of input reports in this application. */
#define INPUT_REPORT_COUNT	   3
/* Length of Mouse Input Report containing button data. */
#define INPUT_REP_BUTTONS_LEN	   3
/* Length of Mouse Input Report containing movement data. */
#define INPUT_REP_MOVEMENT_LEN	   3
/* Length of Mouse Input Report containing media player data. */
#define INPUT_REP_MEDIA_PLAYER_LEN 1
/* Index of Mouse Input Report containing button data. */
#define INPUT_REP_BUTTONS_INDEX	   0
/* Index of Mouse Input Report containing movement data. */
#define INPUT_REP_MOVEMENT_INDEX   1
/* Index of Mouse Input Report containing media player data. */
#define INPUT_REP_MPLAYER_INDEX	   2
/* Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_BUTTONS_ID   1
/* Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MOVEMENT_ID  2
/* Id of reference to Mouse Input Report containing media player data. */
#define INPUT_REP_REF_MPLAYER_ID   3

/* HIDs queue size. */
#define HIDS_QUEUE_SIZE 10
/* Key used to move cursor left */
#define KEY_LEFT_MASK	DK_BTN1_MSK
/* Key used to move cursor up */
#define KEY_UP_MASK	DK_BTN2_MSK
/* Key used to move cursor right */
#define KEY_RIGHT_MASK	DK_BTN3_MSK
/* Key used to move cursor down */
#define KEY_DOWN_MASK	DK_BTN4_MSK

/* Key used to accept or reject passkey value */
#define KEY_PAIRING_ACCEPT DK_BTN1_MSK
#define KEY_PAIRING_REJECT DK_BTN2_MSK

/* HIDS instance. */
BT_HIDS_DEF(hids_obj, INPUT_REP_BUTTONS_LEN, INPUT_REP_MOVEMENT_LEN, INPUT_REP_MEDIA_PLAYER_LEN);

static const struct device *gpio;
static struct k_work hids_work;
#define erase_bond_btn 4 //P0.04

static struct bt_le_ext_adv *adv;
static struct k_work adv_work;
struct mouse_pos {
	int16_t x_val;
	int16_t y_val;
};

/* Mouse movement queue. */
K_MSGQ_DEFINE(hids_queue, sizeof(struct mouse_pos), HIDS_QUEUE_SIZE, 4);

#define AVAILABLE_SINK_CONTEXT	 BT_AUDIO_CONTEXT_TYPE_ANY
#define AVAILABLE_SOURCE_CONTEXT BT_AUDIO_CONTEXT_TYPE_ANY

NET_BUF_POOL_FIXED_DEFINE(tx_pool, CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static struct conn_mode {
	struct bt_conn *conn;
	bool in_boot_mode;
} conn_mode[CONFIG_BT_HIDS_MAX_CLIENT_COUNT];


#define GAIN_DEFAULT	      0x50
#define MAX_SAMPLE_RATE	      32000
#define MAX_FRAME_DURATION_US 10000
#define MAX_NUM_SAMPLES	      ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)
#define TOTAL_BUF_NEEDED      4
static K_SEM_DEFINE(lc3_encoder_sem, 0U, TOTAL_BUF_NEEDED);
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT	 1000
/* Size of a block for 10 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels)                                              \
	(BYTES_PER_SAMPLE * (_sample_rate / 100) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE BLOCK_SIZE(MAX_SAMPLE_RATE, 4)
#define BLOCK_COUNT    4
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);
static const struct device *const i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s20));

static int16_t send_pcm_data[MAX_NUM_SAMPLES];
static const struct bt_audio_codec_cap lc3_codec_cap = BT_AUDIO_CODEC_CAP_LC3(
	(BT_AUDIO_CODEC_CAP_FREQ_16KHZ|BT_AUDIO_CODEC_CAP_FREQ_32KHZ), BT_AUDIO_CODEC_CAP_DURATION_10,
	BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1), 40u, 120u, 1u, BT_AUDIO_CONTEXT_TYPE_ANY);

static struct bt_conn *default_conn;
static struct bt_bap_stream sink_streams[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static struct audio_source {
	struct bt_bap_stream stream;
	uint16_t seq_num;
	uint16_t max_sdu;
	size_t len_to_send;
} source_streams[CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT];
static int configured_octets_per_frame;
static size_t configured_source_stream_count;

static const struct bt_bap_qos_cfg_pref qos_pref =
	BT_BAP_QOS_CFG_PREF(true, BT_GAP_LE_PHY_2M, 0x02, 10, 10000, 40000, 10000, 40000);

static uint8_t unicast_server_addata[] = {
	BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),	/* ASCS UUID */
	BT_AUDIO_UNICAST_ANNOUNCEMENT_TARGETED, /* Target Announcement */
	BT_BYTES_LIST_LE16(AVAILABLE_SINK_CONTEXT),
	BT_BYTES_LIST_LE16(AVAILABLE_SOURCE_CONTEXT),
	0x00, /* Metadata length */
};

/* TODO: Expand with BAP data */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL)),
	BT_DATA(BT_DATA_SVC_DATA16, unicast_server_addata, ARRAY_SIZE(unicast_server_addata)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static uint16_t configured_sampling_freq;
static int i2s_mic_init(uint16_t sampling_rate);

#define AUDIO_VOLUME		(INT16_MAX - 3000) /* codec does clipping above INT16_MAX - 3000 */
#define AUDIO_TONE_FREQUENCY_HZ 400

#define ACL_LINK_STATUS	  DK_LED1
#define ISO_STREAM_STATUS DK_LED2
#define ADV_STATUS	  	  DK_LED3

#define LC3_ENCODER_STACK_SIZE 8192
#define LC3_ENCODER_PRIORITY   2
static void i2s_fetch_thread(void *arg1, void *arg2, void *arg3);
K_THREAD_DEFINE(i2s_fetch, LC3_ENCODER_STACK_SIZE, i2s_fetch_thread, NULL, NULL, NULL,
		LC3_ENCODER_PRIORITY, 0, -1);

static uint16_t get_and_incr_seq_num(const struct bt_bap_stream *stream)
{
	for (size_t i = 0U; i < configured_source_stream_count; i++) {
		if (stream == &source_streams[i].stream) {
			uint16_t seq_num;

			seq_num = source_streams[i].seq_num;
			source_streams[i].seq_num++;

			return seq_num;
		}
	}

	LOG_INF("Could not find endpoint from stream %p", stream);

	return 0;
}

void print_hex(const uint8_t *ptr, size_t len)
{
	while (len-- != 0) {
		LOG_INF("%02x", *ptr++);
	}
}

static void print_codec_cfg(const struct bt_audio_codec_cfg *codec_cfg)
{
	LOG_INF("codec_cfg 0x%02x cid 0x%04x vid 0x%04x count %u", codec_cfg->id, codec_cfg->cid,
		codec_cfg->vid, codec_cfg->data_len);

	if (codec_cfg->id == BT_HCI_CODING_FORMAT_LC3) {
		enum bt_audio_location chan_allocation;
		int ret;

		ret = bt_audio_codec_cfg_get_freq(codec_cfg);
		if (ret > 0) {
			configured_sampling_freq = bt_audio_codec_cfg_freq_to_freq_hz(ret);
			LOG_INF("  Frequency: %d Hz", configured_sampling_freq);
		}

		ret = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
		if (ret > 0) {
			LOG_INF("  Frame Duration: %d us",
				bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret));
		}

		ret = bt_audio_codec_cfg_get_chan_allocation(codec_cfg, &chan_allocation, false);
		if (ret == 0) {
			LOG_INF("  Channel allocation: 0x%x", chan_allocation);
		}

		LOG_INF("  Octets per frame: %d (negative means value not present)",
			bt_audio_codec_cfg_get_octets_per_frame(codec_cfg));
		LOG_INF("  Frames per SDU: %d",
			bt_audio_codec_cfg_get_frame_blocks_per_sdu(codec_cfg, true));
	} else {
		print_hex(codec_cfg->data, codec_cfg->data_len);
	}
}

static void print_qos(const struct bt_bap_qos_cfg *qos)
{
	LOG_INF("QoS: interval %u framing 0x%02x phy 0x%02x sdu %u "
		"rtn %u latency %u pd %u",
		qos->interval, qos->framing, qos->phy, qos->sdu, qos->rtn, qos->latency, qos->pd);
}

static void send_data()
{
	int ret;
	uint8_t lc3_encoded_buffer[CONFIG_BT_ISO_TX_MTU];
	uint16_t encoded_bytes_written;
	struct net_buf *buf;

	/* We configured the sink streams to be first in `streams`, so that
	 * we can use `stream[i]` to select sink streams (i.e. streams with
	 * data going to the server)
	 */
	for (size_t i = 0; i < configured_source_stream_count; i++) {
		struct bt_bap_stream *stream = &source_streams[i].stream;

		buf = net_buf_alloc(&tx_pool, K_FOREVER);
		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
		memset(lc3_encoded_buffer, 0, sizeof(lc3_encoded_buffer));

		ret = sw_codec_lc3_enc_run(
			send_pcm_data, sizeof(send_pcm_data), configured_octets_per_frame * 8 * 100,
			0, sizeof(lc3_encoded_buffer), lc3_encoded_buffer, &encoded_bytes_written);
		if (ret) {
			LOG_INF("LC3 encoder failed - wrong parameters?: %d", ret);
			net_buf_unref(buf);
			return;
		}

		net_buf_add_mem(buf, lc3_encoded_buffer, configured_octets_per_frame);

		ret = bt_bap_stream_send(stream, buf, get_and_incr_seq_num(stream));
		if (ret < 0) {
			LOG_INF("Failed to send audio data on streams[%zu] (%p): (%d)", i, stream,
				ret);
			net_buf_unref(buf);
		}
	}
}

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

static int lc3_config(struct bt_conn *conn, const struct bt_bap_ep *ep, enum bt_audio_dir dir,
		      const struct bt_audio_codec_cfg *codec_cfg, struct bt_bap_stream **stream,
		      struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	int ret;
	uint16_t pcm_bytes_req_enc;

	LOG_INF("ASE Codec Config: conn %p ep %p dir %u", (void *)conn, (void *)ep, dir);

	print_codec_cfg(codec_cfg);

	*stream = stream_alloc(dir);
	if (*stream == NULL) {
		LOG_INF("No streams available");
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_NO_MEM, BT_BAP_ASCS_REASON_NONE);

		return -ENOMEM;
	}

	LOG_INF("ASE Codec Config stream %p", (void *)*stream);

	if (dir == BT_AUDIO_DIR_SOURCE) {
		configured_octets_per_frame = bt_audio_codec_cfg_get_octets_per_frame(codec_cfg);
		ret = sw_codec_lc3_enc_init(configured_sampling_freq, 16, MAX_FRAME_DURATION_US,
					    configured_octets_per_frame * 8 * 100, 1,
					    &pcm_bytes_req_enc);
		if (ret) {
			LOG_INF("sw_codec_lc3_enc_init failed (ret %d)", ret);
		} else {
			LOG_INF("LC3 encoder initialized, PCM bytes required for encoding: %u",
				pcm_bytes_req_enc);
		}
		configured_source_stream_count = 1;
	}

	*pref = qos_pref;

	return 0;
}

static int lc3_reconfig(struct bt_bap_stream *stream, enum bt_audio_dir dir,
			const struct bt_audio_codec_cfg *codec_cfg,
			struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("ASE Codec Reconfig: stream %p", (void *)stream);

	print_codec_cfg(codec_cfg);

	*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_UNSUPPORTED, BT_BAP_ASCS_REASON_NONE);

	/* We only support one QoS at the moment, reject changes */
	return -ENOEXEC;
}

static int lc3_qos(struct bt_bap_stream *stream, const struct bt_bap_qos_cfg *qos,
		   struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("QoS: stream %p qos %p", (void *)stream, (void *)qos);

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
	LOG_INF("Enable: stream %p meta_len %zu", (void *)stream, meta_len);

	return 0;
}

static int lc3_start(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("Start: stream %p", (void *)stream);

	for (size_t i = 0U; i < configured_source_stream_count; i++) {
		if (stream == &source_streams[i].stream) {
			source_streams[i].seq_num = 0U;
			break;
		}
	}

	return 0;
}

static bool data_func_cb(struct bt_data *data, void *user_data)
{
	struct bt_bap_ascs_rsp *rsp = (struct bt_bap_ascs_rsp *)user_data;

	if (!BT_AUDIO_METADATA_TYPE_IS_KNOWN(data->type)) {
		LOG_INF("Invalid metadata type %u or length %u", data->type, data->data_len);
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_METADATA_REJECTED, data->type);

		return false;
	}

	return true;
}

static int lc3_metadata(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len,
			struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("Metadata: stream %p meta_len %zu", (void *)stream, meta_len);

	return bt_audio_data_parse(meta, meta_len, data_func_cb, rsp);
}

static int lc3_disable(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("Disable: stream %p", (void *)stream);

	return 0;
}

static int lc3_stop(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("Stop: stream %p", (void *)stream);
	return 0;
}

static int lc3_release(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{

	if (stream_dir(stream) == BT_AUDIO_DIR_SOURCE) {
		sw_codec_lc3_enc_uninit_all();
	}
	LOG_INF("Release: stream %p", (void *)stream);
	return 0;
}

static struct bt_bap_unicast_server_register_param param = {CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT,
							    CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT};

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

static void stream_recv(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info,
			struct net_buf *buf)
{
	if (info->flags & BT_ISO_FLAGS_VALID) {
		LOG_DBG("Incoming audio on stream %p len %u", (void *)stream, buf->len);
	}
}

static void disconnect_work_handler(struct k_work *work)
{
	bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

K_WORK_DEFINE(work_disconnect, disconnect_work_handler);

static void stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
	int ret;
	LOG_INF("Audio Stream %p stopped with reason 0x%02X", (void *)stream, reason);

	if (stream_dir(stream) == BT_AUDIO_DIR_SOURCE) {
		dk_set_led_off(ISO_STREAM_STATUS);
		k_thread_suspend(i2s_fetch);
		ret = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_DROP);
		if (ret < 0) {
			LOG_INF("I2S stop trigger failed: %d", ret);
		} else {
			LOG_INF("I2S stop trigger success");
		}
	}

	/* Workaround for unexpected disconnection
	 * If ISO disconnected due to timeout, disconnect the ACL connection for central to
	 * re-establish the link and stream.
	 */
	if (reason == 0x08) {
		k_work_submit(&work_disconnect);
	}
}

static void stream_started(struct bt_bap_stream *stream)
{
	int ret;
	LOG_INF("Audio Stream %p started", (void *)stream);

	if (stream_dir(stream) == BT_AUDIO_DIR_SOURCE) {
		dk_set_led_on(ISO_STREAM_STATUS);
		k_thread_resume(i2s_fetch);
		ret = i2s_mic_init(configured_sampling_freq);
		if (ret) {
			LOG_ERR("Cannot init I2S mic: %d", ret);
		}
		ret = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
		if (ret < 0) {
			LOG_INF("I2S start trigger failed: %d", ret);
		} else {
			LOG_INF("I2S start trigger success");
			k_sem_give(&lc3_encoder_sem);
		}
	}
}

static void stream_enabled_cb(struct bt_bap_stream *stream)
{
	/* The unicast server is responsible for starting sink ASEs after the
	 * client has enabled them.
	 */
	if (stream_dir(stream) == BT_AUDIO_DIR_SINK) {
		const int err = bt_bap_stream_start(stream);

		if (err != 0) {
			LOG_INF("Failed to start stream %p: %d", (void *)stream, err);
		}
	}
}

static void stream_disabled_cb(struct bt_bap_stream *stream)
{
}

static void stream_sent_cb(struct bt_bap_stream *stream)
{
	static uint32_t sent_num;
	sent_num++;
	if (sent_num % 100 == 0) {
		LOG_INF("Sent %u packets", sent_num);
	}
	k_sem_give(&lc3_encoder_sem);
}

static struct bt_bap_stream_ops stream_ops = {
	.recv = stream_recv,
	.stopped = stream_stopped,
	.started = stream_started,
	.enabled = stream_enabled_cb,
	.disabled = stream_disabled_cb,
	.sent = stream_sent_cb,
};

static void insert_conn_object(struct bt_conn *conn)
{
	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (!conn_mode[i].conn) {
			conn_mode[i].conn = conn;
			conn_mode[i].in_boot_mode = false;

			return;
		}
	}

	printk("Connection object could not be inserted %p\n", conn);
}

static void hids_pm_evt_handler(enum bt_hids_pm_evt evt, struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	size_t i;

	for (i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (conn_mode[i].conn == conn) {
			break;
		}
	}

	if (i >= CONFIG_BT_HIDS_MAX_CLIENT_COUNT) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	switch (evt) {
	case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
		printk("Boot mode entered %s\n", addr);
		conn_mode[i].in_boot_mode = true;
		break;

	case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
		printk("Report mode entered %s\n", addr);
		conn_mode[i].in_boot_mode = false;
		break;

	default:
		break;
	}
}

static void hid_init(void)
{
	int err;
	struct bt_hids_init_param hids_init_param = {0};
	struct bt_hids_inp_rep *hids_inp_rep;
	static const uint8_t mouse_movement_mask[DIV_ROUND_UP(INPUT_REP_MOVEMENT_LEN, 8)] = {0};

	static const uint8_t report_map[] = {
		0x05, 0x01, /* Usage Page (Generic Desktop) */
		0x09, 0x02, /* Usage (Mouse) */

		0xA1, 0x01, /* Collection (Application) */

		/* Report ID 1: Mouse buttons + scroll/pan */
		0x85, 0x01,	  /* Report Id 1 */
		0x09, 0x01,	  /* Usage (Pointer) */
		0xA1, 0x00,	  /* Collection (Physical) */
		0x95, 0x05,	  /* Report Count (3) */
		0x75, 0x01,	  /* Report Size (1) */
		0x05, 0x09,	  /* Usage Page (Buttons) */
		0x19, 0x01,	  /* Usage Minimum (01) */
		0x29, 0x05,	  /* Usage Maximum (05) */
		0x15, 0x00,	  /* Logical Minimum (0) */
		0x25, 0x01,	  /* Logical Maximum (1) */
		0x81, 0x02,	  /* Input (Data, Variable, Absolute) */
		0x95, 0x01,	  /* Report Count (1) */
		0x75, 0x03,	  /* Report Size (3) */
		0x81, 0x01,	  /* Input (Constant) for padding */
		0x75, 0x08,	  /* Report Size (8) */
		0x95, 0x01,	  /* Report Count (1) */
		0x05, 0x01,	  /* Usage Page (Generic Desktop) */
		0x09, 0x38,	  /* Usage (Wheel) */
		0x15, 0x81,	  /* Logical Minimum (-127) */
		0x25, 0x7F,	  /* Logical Maximum (127) */
		0x81, 0x06,	  /* Input (Data, Variable, Relative) */
		0x05, 0x0C,	  /* Usage Page (Consumer) */
		0x0A, 0x38, 0x02, /* Usage (AC Pan) */
		0x95, 0x01,	  /* Report Count (1) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0xC0,		  /* End Collection (Physical) */

		/* Report ID 2: Mouse motion */
		0x85, 0x02,	  /* Report Id 2 */
		0x09, 0x01,	  /* Usage (Pointer) */
		0xA1, 0x00,	  /* Collection (Physical) */
		0x75, 0x0C,	  /* Report Size (12) */
		0x95, 0x02,	  /* Report Count (2) */
		0x05, 0x01,	  /* Usage Page (Generic Desktop) */
		0x09, 0x30,	  /* Usage (X) */
		0x09, 0x31,	  /* Usage (Y) */
		0x16, 0x01, 0xF8, /* Logical maximum (2047) */
		0x26, 0xFF, 0x07, /* Logical minimum (-2047) */
		0x81, 0x06,	  /* Input (Data, Variable, Relative) */
		0xC0,		  /* End Collection (Physical) */
		0xC0,		  /* End Collection (Application) */

		/* Report ID 3: Advanced buttons */
		0x05, 0x0C, /* Usage Page (Consumer) */
		0x09, 0x01, /* Usage (Consumer Control) */
		0xA1, 0x01, /* Collection (Application) */
		0x85, 0x03, /* Report Id (3) */
		0x15, 0x00, /* Logical minimum (0) */
		0x25, 0x01, /* Logical maximum (1) */
		0x75, 0x01, /* Report Size (1) */
		0x95, 0x01, /* Report Count (1) */

		0x09, 0xCD,	  /* Usage (Play/Pause) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0x0A, 0x83, 0x01, /* Usage (Consumer Control Configuration) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0x09, 0xB5,	  /* Usage (Scan Next Track) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0x09, 0xB6,	  /* Usage (Scan Previous Track) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */

		0x09, 0xEA,	  /* Usage (Volume Down) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0x09, 0xE9,	  /* Usage (Volume Up) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0x0A, 0x25, 0x02, /* Usage (AC Forward) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0x0A, 0x24, 0x02, /* Usage (AC Back) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0xC0		  /* End Collection */
	};

	hids_init_param.rep_map.data = report_map;
	hids_init_param.rep_map.size = sizeof(report_map);

	hids_init_param.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
	hids_init_param.info.b_country_code = 0x00;
	hids_init_param.info.flags = (BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE);

	hids_inp_rep = &hids_init_param.inp_rep_group_init.reports[0];
	hids_inp_rep->size = INPUT_REP_BUTTONS_LEN;
	hids_inp_rep->id = INPUT_REP_REF_BUTTONS_ID;
	hids_init_param.inp_rep_group_init.cnt++;

	hids_inp_rep++;
	hids_inp_rep->size = INPUT_REP_MOVEMENT_LEN;
	hids_inp_rep->id = INPUT_REP_REF_MOVEMENT_ID;
	hids_inp_rep->rep_mask = mouse_movement_mask;
	hids_init_param.inp_rep_group_init.cnt++;

	hids_inp_rep++;
	hids_inp_rep->size = INPUT_REP_MEDIA_PLAYER_LEN;
	hids_inp_rep->id = INPUT_REP_REF_MPLAYER_ID;
	hids_init_param.inp_rep_group_init.cnt++;

	hids_init_param.is_mouse = true;
	hids_init_param.pm_evt_handler = hids_pm_evt_handler;

	err = bt_hids_init(&hids_obj, &hids_init_param);
	__ASSERT(err == 0, "HIDS initialization failed\n");
}

static void mouse_movement_send(int16_t x_delta, int16_t y_delta)
{
	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {

		if (!conn_mode[i].conn) {
			continue;
		}

		if (conn_mode[i].in_boot_mode) {
			x_delta = MAX(MIN(x_delta, SCHAR_MAX), SCHAR_MIN);
			y_delta = MAX(MIN(y_delta, SCHAR_MAX), SCHAR_MIN);

			bt_hids_boot_mouse_inp_rep_send(&hids_obj, conn_mode[i].conn, NULL,
							(int8_t)x_delta, (int8_t)y_delta, NULL);
		} else {
			uint8_t x_buff[2];
			uint8_t y_buff[2];
			uint8_t buffer[INPUT_REP_MOVEMENT_LEN];

			int16_t x = MAX(MIN(x_delta, 0x07ff), -0x07ff);
			int16_t y = MAX(MIN(y_delta, 0x07ff), -0x07ff);

			/* Convert to little-endian. */
			sys_put_le16(x, x_buff);
			sys_put_le16(y, y_buff);

			/* Encode report. */
			BUILD_ASSERT(sizeof(buffer) == 3,
				     "Only 2 axis, 12-bit each, are supported");

			buffer[0] = x_buff[0];
			buffer[1] = (y_buff[0] << 4) | (x_buff[1] & 0x0f);
			buffer[2] = (y_buff[1] << 4) | (y_buff[0] >> 4);

			bt_hids_inp_rep_send(&hids_obj, conn_mode[i].conn, INPUT_REP_MOVEMENT_INDEX,
					     buffer, sizeof(buffer), NULL);
		}
	}
}

static void mouse_handler(struct k_work *work)
{
	struct mouse_pos pos;

	while (!k_msgq_get(&hids_queue, &pos, K_NO_WAIT)) {
		mouse_movement_send(pos.x_val, pos.y_val);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err != 0) {
		LOG_INF("Failed to connect to %s %u %s", addr, err, bt_hci_err_to_str(err));

		default_conn = NULL;
		return;
	}
	err = bt_hids_connected(&hids_obj, conn);

	if (err) {
		printk("Failed to notify HID service about connection\n");
		return;
	}

	insert_conn_object(conn);
	LOG_INF("Connected: %s", addr);
	default_conn = bt_conn_ref(conn);
	dk_set_led_on(ACL_LINK_STATUS);
	dk_set_led_off(ADV_STATUS);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;
	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	configured_source_stream_count = 0U;

	LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

	bt_conn_unref(default_conn);
	default_conn = NULL;
	dk_set_led_off(ACL_LINK_STATUS);

	err = bt_hids_disconnected(&hids_obj, conn);

	if (err) {
		printk("Failed to notify HID service about disconnection\n");
	}

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (conn_mode[i].conn == conn) {
			conn_mode[i].conn = NULL;
			break;
		}
	}
	k_work_submit(&adv_work);
}

static void security_level_changed(struct bt_conn *conn, bt_security_t level,
				   enum bt_security_err err)
{
	LOG_WRN("security_level_changed to %d, err %d", level, err);
	if (err == BT_SECURITY_ERR_AUTH_REQUIREMENT) {
		bt_unpair(BT_ID_DEFAULT, bt_conn_get_dst(conn));
		bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_level_changed,
};

static struct bt_pacs_cap cap_sink = {
	.codec_cap = &lc3_codec_cap,
};

static struct bt_pacs_cap cap_source = {
	.codec_cap = &lc3_codec_cap,
};

static int set_location(void)
{
	int err;

	if (IS_ENABLED(CONFIG_BT_PAC_SNK_LOC)) {
		err = bt_pacs_set_location(BT_AUDIO_DIR_SINK, BT_AUDIO_LOCATION_FRONT_LEFT);
		if (err != 0) {
			LOG_INF("Failed to set sink location (err %d)", err);
			return err;
		}
	}

	if (IS_ENABLED(CONFIG_BT_PAC_SRC_LOC)) {
		err = bt_pacs_set_location(BT_AUDIO_DIR_SOURCE, BT_AUDIO_LOCATION_FRONT_LEFT);
		if (err != 0) {
			LOG_INF("Failed to set source location (err %d)", err);
			return err;
		}
	}

	LOG_INF("Location successfully set");

	return 0;
}

static int set_supported_contexts(void)
{
	int err;

	if (IS_ENABLED(CONFIG_BT_PAC_SNK)) {
		err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SINK, AVAILABLE_SINK_CONTEXT);
		if (err != 0) {
			LOG_INF("Failed to set sink supported contexts (err %d)", err);

			return err;
		}
	}

	if (IS_ENABLED(CONFIG_BT_PAC_SRC)) {
		err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SOURCE, AVAILABLE_SOURCE_CONTEXT);
		if (err != 0) {
			LOG_INF("Failed to set source supported contexts (err %d)", err);

			return err;
		}
	}

	LOG_INF("Supported contexts successfully set");

	return 0;
}

static int set_available_contexts(void)
{
	int err;

	if (IS_ENABLED(CONFIG_BT_PAC_SNK)) {
		err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SINK, AVAILABLE_SINK_CONTEXT);
		if (err != 0) {
			LOG_INF("Failed to set sink available contexts (err %d)", err);
			return err;
		}
	}

	if (IS_ENABLED(CONFIG_BT_PAC_SRC)) {
		err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SOURCE, AVAILABLE_SOURCE_CONTEXT);
		if (err != 0) {
			LOG_INF("Failed to set source available contexts (err %d)", err);
			return err;
		}
	}

	LOG_INF("Available contexts successfully set");
	return 0;
}

static int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_INF("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_INF("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_INF("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	LOG_INF("HF clock started");
	return 0;
}

static void i2s_fetch_thread(void *arg1, void *arg2, void *arg3)
{
	int ret;
	void *buffer;
	size_t size;

	while (true) {
		k_sem_take(&lc3_encoder_sem, K_FOREVER);

		ret = i2s_read(i2s_dev, &buffer, &size);
		if (ret == -5) {
			LOG_INF("I2S read failed: %d", ret);
			i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_PREPARE);
			//i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
		}else if (ret == -11){
			LOG_INF("I2S read failed: %d", ret);
			i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
			//i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
		}
		if (size > sizeof(send_pcm_data)) {
			LOG_INF("Buffer size exceeds send_pcm_data size");
			size = sizeof(send_pcm_data);
		}
		memcpy(send_pcm_data, buffer, size);

		k_mem_slab_free(&mem_slab, buffer);
		send_data();
	}
}

static int i2s_mic_init(uint16_t sampling_rate)
{
	int err;
	struct i2s_config i2s_cfg;

	if (!device_is_ready(i2s_dev)) {
		LOG_ERR("I2S device not ready");
		return -ENODEV;
	}

	i2s_cfg.word_size = SAMPLE_BIT_WIDTH;
	i2s_cfg.channels = 1;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	i2s_cfg.frame_clk_freq = sampling_rate;
	i2s_cfg.mem_slab = &mem_slab;
	i2s_cfg.block_size = BLOCK_SIZE(sampling_rate, 1);
	i2s_cfg.timeout = READ_TIMEOUT;

	err = i2s_configure(i2s_dev, I2S_DIR_RX, &i2s_cfg);
	if (err < 0) {
		LOG_INF("Failed to configure the I2S driver: %d", err);
		return err;
	}

	return 0;
}

/* Handles button state changes and adjusts PDM gain accordingly */
static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	bool data_to_send = false;
	struct mouse_pos pos;
	uint32_t buttons = button_state & has_changed;

	memset(&pos, 0, sizeof(struct mouse_pos));

	if (buttons & KEY_LEFT_MASK) {
		pos.x_val -= MOVEMENT_SPEED;
		printk("%s(): left\n", __func__);
		data_to_send = true;
	}
	if (buttons & KEY_UP_MASK) {
		pos.y_val -= MOVEMENT_SPEED;
		printk("%s(): up\n", __func__);
		data_to_send = true;
	}
	if (buttons & KEY_RIGHT_MASK) {
		pos.x_val += MOVEMENT_SPEED;
		printk("%s(): right\n", __func__);
		data_to_send = true;
	}
	if (buttons & KEY_DOWN_MASK) {
		pos.y_val += MOVEMENT_SPEED;
		printk("%s(): down\n", __func__);
		data_to_send = true;
	}

	if (data_to_send) {
		int err;

		err = k_msgq_put(&hids_queue, &pos, K_NO_WAIT);
		if (err) {
			printk("No space in the queue for button pressed\n");
			return;
		}
		if (k_msgq_num_used_get(&hids_queue) == 1) {
			k_work_submit(&hids_work);
		}
	}
}

static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 100U;
	}

	bt_bas_set_battery_level(battery_level);
}

static void advertising_process(struct k_work *work)
{
	int err;
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		LOG_INF("Failed to start advertising set (err %d)", err);
	}
	LOG_INF("Advertising successfully started");
	dk_set_led_on(ADV_STATUS);
}

int main(void)
{
	int err, ret;

	err = clocks_start();
	if (err) {
		LOG_INF("Failed to start clocks (err %d)", err);
		return 0;
	}

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}

	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}

	/* DIS initialized at system boot with SYS_INIT macro. */
	hid_init();

	err = bt_enable(NULL);
	if (err != 0) {
		LOG_INF("Bluetooth init failed (err %d)", err);
		return 0;
	}
	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	gpio = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	ret = gpio_pin_get(gpio, erase_bond_btn);
	if (ret == 1) {
		if (IS_ENABLED(CONFIG_SETTINGS)) {
			LOG_INF("Clearing all bonds");

			ret = bt_unpair(BT_ID_DEFAULT, NULL);
			if (ret) {
				LOG_ERR("Failed to clear bonding: %d", ret);
				return ret;
			}
		}
	}

	k_work_init(&hids_work, mouse_handler);
	err = sw_codec_lc3_init(NULL, NULL, MAX_FRAME_DURATION_US);
	if (err) {
		LOG_INF("sw_codec_lc3_init failed (err %d)", err);
	}

	bt_bap_unicast_server_register(&param);
	bt_bap_unicast_server_register_cb(&unicast_server_cb);

	bt_pacs_cap_register(BT_AUDIO_DIR_SINK, &cap_sink);
	bt_pacs_cap_register(BT_AUDIO_DIR_SOURCE, &cap_source);

	for (size_t i = 0; i < ARRAY_SIZE(sink_streams); i++) {
		bt_bap_stream_cb_register(&sink_streams[i], &stream_ops);
	}

	for (size_t i = 0; i < ARRAY_SIZE(source_streams); i++) {
		bt_bap_stream_cb_register(&source_streams[i].stream, &stream_ops);
	}

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

	/* Create a connectable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_CONN, NULL, &adv);
	if (err) {
		LOG_INF("Failed to create advertising set (err %d)", err);
		return 0;
	}

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_INF("Failed to set advertising data (err %d)", err);
		return 0;
	}

	k_thread_start(i2s_fetch);

	k_work_init(&adv_work, advertising_process);
	k_work_submit(&adv_work);

	while (true) {
		k_sleep(K_SECONDS(1));
		bas_notify();		
	}
	return 0;
}
