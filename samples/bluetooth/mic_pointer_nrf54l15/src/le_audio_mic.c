/*
 * Copyright (c) 2021-2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * LE Audio unicast server exposing the on-board PDM microphone as an audio
 * source: PDM -> LC3 (16 kHz, 7.5 ms frames, mono) -> ISO.
 *
 * Adapted from the bap_unicast_server_nrf54l15 sample. The XIAO nRF54L15 Sense
 * carries the PDM microphone on the same pins as the DK overlay used there
 * (P1.12 CLK, P1.13 DIN), and its devicetree already enables pdm20 as
 * "dmic_dev", so no overlay is needed.
 */

#include <errno.h>
#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/micp.h>
#include <zephyr/bluetooth/audio/pacs.h>

#include <hal/nrf_clock.h>
#include <hal/nrf_pdm.h>

#include "sw_codec_lc3.h"
#include "le_audio_mic.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(le_audio_mic, LOG_LEVEL_INF);

/* ------------------------------------------------------------------------- */
/* Audio format                                                              */
/* ------------------------------------------------------------------------- */

#define SAMPLE_RATE_HZ	   16000
#define FRAME_DURATION_US  7500
#define SAMPLE_BIT_WIDTH   16
#define BYTES_PER_SAMPLE   sizeof(int16_t)
#define NUM_SAMPLES	   ((FRAME_DURATION_US * SAMPLE_RATE_HZ) / USEC_PER_SEC)

/* PDM gain, 0x00..0x50. The DK sample tunes this with buttons; the XIAO has a
 * single button and it is the mouse click, so pick a value at build time.
 */
#define PDM_MIC_GAIN NRF_PDM_GAIN_DEFAULT

/* Clear the MICS mute when a stream starts. Set to 0 to leave whatever mute
 * state the host asked for untouched.
 */
#define LE_AUDIO_MIC_UNMUTE_ON_START 1

/* Size of one 7.5 ms block (the driver works in 10 ms units, hence the 75%). */
#define BLOCK_SIZE(_sample_rate, _number_of_channels)                                              \
	((BYTES_PER_SAMPLE * (_sample_rate / 100) * _number_of_channels) * 75 / 100)

#define MAX_BLOCK_SIZE BLOCK_SIZE(SAMPLE_RATE_HZ, 2)
#define BLOCK_COUNT    8

/* Blocks the DMIC driver fills and the fetch thread frees. */
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 8);

NET_BUF_POOL_FIXED_DEFINE(tx_pool, CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

/* One credit per buffer that may be in flight towards the controller. */
#define TOTAL_BUF_NEEDED 4
static K_SEM_DEFINE(lc3_encoder_sem, 0U, TOTAL_BUF_NEEDED);

#define LC3_ENCODER_STACK_SIZE 8192
#define LC3_ENCODER_PRIORITY   5

static void dmic_fetch_thread(void *arg1, void *arg2, void *arg3);
K_THREAD_DEFINE(dmic_fetch, LC3_ENCODER_STACK_SIZE, dmic_fetch_thread, NULL, NULL, NULL,
		LC3_ENCODER_PRIORITY, 0, -1);

static const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
static int16_t send_pcm_data[NUM_SAMPLES];

/* ------------------------------------------------------------------------- */
/* BAP state                                                                 */
/* ------------------------------------------------------------------------- */

static const struct bt_audio_codec_cap lc3_codec_cap = BT_AUDIO_CODEC_CAP_LC3(
	BT_AUDIO_CODEC_CAP_FREQ_16KHZ, BT_AUDIO_CODEC_CAP_DURATION_7_5,
	BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1), 30u, 120u, 1u, BT_AUDIO_CONTEXT_TYPE_ANY);

static struct audio_source {
	struct bt_bap_stream stream;
	uint16_t seq_num;
	uint16_t max_sdu;
} source_streams[CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT];

static int configured_octets_per_frame;
static size_t configured_source_stream_count;

static const struct bt_bap_qos_cfg_pref qos_pref =
	BT_BAP_QOS_CFG_PREF(true, BT_GAP_LE_PHY_2M, 0x02, 10, 10000, 40000, 10000, 40000);

static struct bt_conn *audio_conn;

/* Set from the MICS callback, read by the capture thread. */
static atomic_t mic_muted;

static uint16_t get_and_incr_seq_num(const struct bt_bap_stream *stream)
{
	for (size_t i = 0U; i < configured_source_stream_count; i++) {
		if (stream == &source_streams[i].stream) {
			return source_streams[i].seq_num++;
		}
	}

	LOG_INF("Could not find endpoint from stream %p", (void *)stream);

	return 0;
}

static void print_codec_cfg(const struct bt_audio_codec_cfg *codec_cfg)
{
	int ret;

	if (codec_cfg->id != BT_HCI_CODING_FORMAT_LC3) {
		LOG_INF("codec_cfg 0x%02x (not LC3)", codec_cfg->id);
		return;
	}

	ret = bt_audio_codec_cfg_get_freq(codec_cfg);
	if (ret > 0) {
		LOG_INF("  Frequency: %d Hz", bt_audio_codec_cfg_freq_to_freq_hz(ret));
	}

	ret = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
	if (ret > 0) {
		LOG_INF("  Frame duration: %d us",
			bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret));
	}

	LOG_INF("  Octets per frame: %d", bt_audio_codec_cfg_get_octets_per_frame(codec_cfg));
}

/* Encodes the most recent PCM block and pushes it onto every source stream. */
static void send_data(void)
{
	uint8_t lc3_encoded_buffer[CONFIG_BT_ISO_TX_MTU];
	uint16_t encoded_bytes_written;
	struct net_buf *buf;
	int ret;

	for (size_t i = 0; i < configured_source_stream_count; i++) {
		struct bt_bap_stream *stream = &source_streams[i].stream;

		buf = net_buf_alloc(&tx_pool, K_FOREVER);
		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

		ret = sw_codec_lc3_enc_run(send_pcm_data, sizeof(send_pcm_data),
					   configured_octets_per_frame * 8 * 100 * 100 / 75, 0,
					   sizeof(lc3_encoded_buffer), lc3_encoded_buffer,
					   &encoded_bytes_written);
		if (ret) {
			LOG_ERR("LC3 encoder failed - wrong parameters?: %d", ret);
			net_buf_unref(buf);
			return;
		}

		net_buf_add_mem(buf, lc3_encoded_buffer, configured_octets_per_frame);

		ret = bt_bap_stream_send(stream, buf, get_and_incr_seq_num(stream));
		if (ret < 0) {
			LOG_WRN("Failed to send audio data on stream %p: %d", (void *)stream, ret);
			net_buf_unref(buf);
		}
	}
}

static struct bt_bap_stream *stream_alloc(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(source_streams); i++) {
		if (!source_streams[i].stream.conn) {
			return &source_streams[i].stream;
		}
	}

	return NULL;
}

static int lc3_config(struct bt_conn *conn, const struct bt_bap_ep *ep, enum bt_audio_dir dir,
		      const struct bt_audio_codec_cfg *codec_cfg, struct bt_bap_stream **stream,
		      struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	uint16_t pcm_bytes_req_enc;
	int ret;

	LOG_INF("ASE codec config: conn %p ep %p dir %u", (void *)conn, (void *)ep, dir);
	print_codec_cfg(codec_cfg);

	/* Microphone only - there is nothing to render a sink stream to. */
	if (dir != BT_AUDIO_DIR_SOURCE) {
		LOG_WRN("Rejecting sink stream, this device is a source only");
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_UNSUPPORTED,
				       BT_BAP_ASCS_REASON_NONE);

		return -ENOTSUP;
	}

	*stream = stream_alloc();
	if (*stream == NULL) {
		LOG_WRN("No streams available");
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_NO_MEM, BT_BAP_ASCS_REASON_NONE);

		return -ENOMEM;
	}

	configured_octets_per_frame = bt_audio_codec_cfg_get_octets_per_frame(codec_cfg);

	ret = sw_codec_lc3_enc_init(SAMPLE_RATE_HZ, SAMPLE_BIT_WIDTH, FRAME_DURATION_US,
				    configured_octets_per_frame * 8 * 100 * 100 / 75, 1,
				    &pcm_bytes_req_enc);
	if (ret) {
		LOG_ERR("sw_codec_lc3_enc_init failed (ret %d)", ret);
	} else {
		LOG_INF("LC3 encoder ready, %u PCM bytes per frame", pcm_bytes_req_enc);
	}

	configured_source_stream_count = 1;

	*pref = qos_pref;

	return 0;
}

static int lc3_reconfig(struct bt_bap_stream *stream, enum bt_audio_dir dir,
			const struct bt_audio_codec_cfg *codec_cfg,
			struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("ASE codec reconfig: stream %p", (void *)stream);

	*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_UNSUPPORTED, BT_BAP_ASCS_REASON_NONE);

	/* Only one QoS is supported, reject changes. */
	return -ENOEXEC;
}

static int lc3_qos(struct bt_bap_stream *stream, const struct bt_bap_qos_cfg *qos,
		   struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("QoS: interval %u framing 0x%02x phy 0x%02x sdu %u rtn %u latency %u pd %u",
		qos->interval, qos->framing, qos->phy, qos->sdu, qos->rtn, qos->latency, qos->pd);

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
	LOG_INF("Enable: stream %p", (void *)stream);

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
		LOG_WRN("Invalid metadata type %u or length %u", data->type, data->data_len);
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_METADATA_REJECTED, data->type);

		return false;
	}

	return true;
}

static int lc3_metadata(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len,
			struct bt_bap_ascs_rsp *rsp)
{
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
	LOG_INF("Release: stream %p", (void *)stream);

	return 0;
}

static struct bt_bap_unicast_server_register_param unicast_server_param = {
	CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT, CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT};

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

/* ------------------------------------------------------------------------- */
/* Stream callbacks                                                          */
/* ------------------------------------------------------------------------- */

static void disconnect_work_handler(struct k_work *work)
{
	if (audio_conn) {
		bt_conn_disconnect(audio_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
}

K_WORK_DEFINE(work_disconnect, disconnect_work_handler);

static void stream_started(struct bt_bap_stream *stream)
{
	int ret;

	LOG_INF("Audio stream %p started", (void *)stream);

	if (LE_AUDIO_MIC_UNMUTE_ON_START) {
		/*
		 * Some hosts leave MICS muted by default, which would make the
		 * stream carry nothing but silence with no obvious cause. Clear
		 * the mute as the stream starts. This is a local write, so it is
		 * accepted even when remote mute changes are disabled, and the
		 * client is notified of the resulting state.
		 */
		ret = bt_micp_mic_dev_unmute();
		if (ret) {
			LOG_WRN("Failed to unmute microphone (err %d)", ret);
		}
	}

	k_thread_resume(dmic_fetch);

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("DMIC start trigger failed: %d", ret);
	} else {
		LOG_INF("Microphone streaming");
		k_sem_give(&lc3_encoder_sem);
	}
}

static void stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
	int ret;

	LOG_INF("Audio stream %p stopped, reason 0x%02x", (void *)stream, reason);

	k_thread_suspend(dmic_fetch);

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
	if (ret < 0) {
		LOG_ERR("DMIC stop trigger failed: %d", ret);
	}

	sw_codec_lc3_enc_uninit_all();

	/* If the ISO link timed out, drop the ACL so the client re-establishes
	 * both the connection and the stream.
	 */
	if (reason == BT_HCI_ERR_CONN_TIMEOUT) {
		k_work_submit(&work_disconnect);
	}
}

static void stream_sent_cb(struct bt_bap_stream *stream)
{
	k_sem_give(&lc3_encoder_sem);
}

static struct bt_bap_stream_ops stream_ops = {
	.started = stream_started,
	.stopped = stream_stopped,
	.sent = stream_sent_cb,
};

/* ------------------------------------------------------------------------- */
/* Connection tracking                                                       */
/* ------------------------------------------------------------------------- */

static void audio_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		return;
	}

	audio_conn = bt_conn_ref(conn);
}

static void audio_disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (conn != audio_conn) {
		return;
	}

	bt_conn_unref(audio_conn);
	audio_conn = NULL;
	configured_source_stream_count = 0U;
}

static void audio_security_changed(struct bt_conn *conn, bt_security_t level,
				   enum bt_security_err err)
{
	/* A stale bond on either side leaves the client unable to read ASCS;
	 * drop the bond so the next attempt pairs cleanly.
	 */
	if (err == BT_SECURITY_ERR_AUTH_REQUIREMENT) {
		LOG_WRN("Authentication requirement failed, unpairing");
		bt_unpair(BT_ID_DEFAULT, bt_conn_get_dst(conn));
		bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
	}
}

BT_CONN_CB_DEFINE(audio_conn_callbacks) = {
	.connected = audio_connected,
	.disconnected = audio_disconnected,
	.security_changed = audio_security_changed,
};

/* ------------------------------------------------------------------------- */
/* PACS                                                                      */
/* ------------------------------------------------------------------------- */

static struct bt_pacs_cap cap_source = {
	.codec_cap = &lc3_codec_cap,
};

static int pacs_configure(void)
{
	int err;

	if (IS_ENABLED(CONFIG_BT_PAC_SRC_LOC)) {
		err = bt_pacs_set_location(BT_AUDIO_DIR_SOURCE, BT_AUDIO_LOCATION_FRONT_LEFT);
		if (err) {
			LOG_ERR("Failed to set source location (err %d)", err);
			return err;
		}
	}

	if (IS_ENABLED(CONFIG_BT_PAC_SRC)) {
		err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SOURCE,
						     LE_AUDIO_MIC_SOURCE_CONTEXT);
		if (err) {
			LOG_ERR("Failed to set source supported contexts (err %d)", err);
			return err;
		}

		err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SOURCE,
						     LE_AUDIO_MIC_SOURCE_CONTEXT);
		if (err) {
			LOG_ERR("Failed to set source available contexts (err %d)", err);
			return err;
		}
	}

	return 0;
}

/* ------------------------------------------------------------------------- */
/* Microphone Control Service                                                */
/* ------------------------------------------------------------------------- */

static void mic_dev_mute_cb(uint8_t mute)
{
	switch (mute) {
	case BT_MICP_MUTE_MUTED:
		atomic_set(&mic_muted, 1);
		LOG_INF("Microphone muted");
		break;
	case BT_MICP_MUTE_UNMUTED:
		atomic_set(&mic_muted, 0);
		LOG_INF("Microphone unmuted");
		break;
	case BT_MICP_MUTE_DISABLED:
		/* Mute cannot be changed remotely; keep capturing. */
		atomic_set(&mic_muted, 0);
		LOG_INF("Microphone mute disabled");
		break;
	default:
		LOG_WRN("Unknown mute state %u", mute);
		break;
	}
}

static struct bt_micp_mic_dev_cb mic_dev_cb = {
	.mute = mic_dev_mute_cb,
};

/* ------------------------------------------------------------------------- */
/* PDM microphone                                                            */
/* ------------------------------------------------------------------------- */

static void dmic_fetch_thread(void *arg1, void *arg2, void *arg3)
{
	void *buffer;
	uint32_t size;
	int ret;

	while (true) {
		k_sem_take(&lc3_encoder_sem, K_FOREVER);

		ret = dmic_read(dmic_dev, 0, &buffer, &size, 10);
		if (ret < 0) {
			LOG_ERR("DMIC read failed: %d", ret);
			continue;
		}

		if (size > sizeof(send_pcm_data)) {
			LOG_WRN("DMIC block %u larger than %zu, truncating", size,
				sizeof(send_pcm_data));
			size = sizeof(send_pcm_data);
		}

		memcpy(send_pcm_data, buffer, size);
		k_mem_slab_free(&mem_slab, buffer);

		/* Transmit silence rather than stopping: the stream keeps its
		 * ISO timing and TX credits, so unmuting is seamless.
		 */
		if (atomic_get(&mic_muted)) {
			memset(send_pcm_data, 0, sizeof(send_pcm_data));
		}

		send_data();
	}
}

static int pdm_mic_init(void)
{
	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			/* Bounds the PDM clock to what the microphone supports. */
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3250000,
			.min_pdm_clk_dc = 40,
			.max_pdm_clk_dc = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};
	int err;

	if (!device_is_ready(dmic_dev)) {
		LOG_ERR("DMIC device %s is not ready", dmic_dev->name);
		return -ENODEV;
	}

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = SAMPLE_RATE_HZ;
	cfg.streams[0].block_size = BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	err = dmic_configure(dmic_dev, &cfg);
	if (err < 0) {
		LOG_ERR("Failed to configure the DMIC driver: %d", err);
		return err;
	}

	nrf_pdm_gain_set(NRF_PDM20_S, PDM_MIC_GAIN, PDM_MIC_GAIN);

	LOG_INF("PDM microphone ready: %d Hz mono, %d us frames, %u byte blocks", SAMPLE_RATE_HZ,
		FRAME_DURATION_US, cfg.streams[0].block_size);

	return 0;
}

/* The PDM peripheral and the LC3 codec both need the HF clock running. */
static int clocks_start(void)
{
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;
	int err;
	int res;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif

	return 0;
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

int le_audio_mic_init(void)
{
	int err;

	err = clocks_start();
	if (err) {
		LOG_ERR("Failed to start clocks (err %d)", err);
		return err;
	}

	err = pdm_mic_init();
	if (err) {
		LOG_ERR("Cannot init PDM mic (err %d)", err);
		return err;
	}

	return 0;
}

int le_audio_mic_start(void)
{
	/* Since NCS v3.1 PACS must be registered before any capability is added. */
	const struct bt_pacs_register_param pacs_param = {
		.src_pac = true,
		.src_loc = true,
	};
	struct bt_micp_mic_dev_register_param mic_dev_param = {
		.cb = &mic_dev_cb,
	};
	int err;

	err = bt_pacs_register(&pacs_param);
	if (err) {
		LOG_ERR("Failed to register PACS (err %d)", err);
		return err;
	}

	err = sw_codec_lc3_init(NULL, NULL, FRAME_DURATION_US);
	if (err) {
		LOG_ERR("sw_codec_lc3_init failed (err %d)", err);
		return err;
	}

	err = bt_bap_unicast_server_register(&unicast_server_param);
	if (err) {
		LOG_ERR("Failed to register unicast server (err %d)", err);
		return err;
	}

	err = bt_bap_unicast_server_register_cb(&unicast_server_cb);
	if (err) {
		LOG_ERR("Failed to register unicast server callbacks (err %d)", err);
		return err;
	}

	err = bt_pacs_cap_register(BT_AUDIO_DIR_SOURCE, &cap_source);
	if (err) {
		LOG_ERR("Failed to register source capability (err %d)", err);
		return err;
	}

	/* MICS is a dynamic GATT service too, so this must also run after
	 * settings_load(). It lets the client mute the microphone.
	 */
	err = bt_micp_mic_dev_register(&mic_dev_param);
	if (err) {
		LOG_ERR("Failed to register MICS (err %d)", err);
		return err;
	}

	for (size_t i = 0; i < ARRAY_SIZE(source_streams); i++) {
		bt_bap_stream_cb_register(&source_streams[i].stream, &stream_ops);
	}

	err = pacs_configure();
	if (err) {
		return err;
	}

	k_thread_start(dmic_fetch);

	LOG_INF("LE Audio microphone ready (MICS mute available)");

	return 0;
}
