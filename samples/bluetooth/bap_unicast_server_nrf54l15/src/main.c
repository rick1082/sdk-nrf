/*
 * Copyright (c) 2021-2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*************************************************************************************************/
/* Includes                                              */
/*************************************************************************************************/
#include <errno.h>
#include <stddef.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/dis.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/types.h>

#include <bluetooth/services/hids.h>
#include <dk_buttons_and_leds.h>
#include <nrfx_i2s.h>

#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */

#include "stylus_hid.h"
#include "sw_codec_lc3.h"

LOG_MODULE_REGISTER(app);

/*************************************************************************************************/
/* Defines                                              */
/*************************************************************************************************/
/* LED Aliases */
#define ACL_LINK_STATUS	  DK_LED1
#define ISO_STREAM_STATUS DK_LED2
#define ADV_STATUS	  DK_LED3

/* Button Masks and Pins */
#define KEY_LEFT_MASK  DK_BTN1_MSK
#define KEY_UP_MASK    DK_BTN2_MSK
#define KEY_RIGHT_MASK DK_BTN3_MSK
#define KEY_DOWN_MASK  DK_BTN4_MSK
#define ERASE_BOND_BTN 4 /* P0.04 */

/* HIDS Configuration */
#define HIDS_QUEUE_SIZE 10

/* Audio Configuration */
#define AVAILABLE_SINK_CONTEXT	 BT_AUDIO_CONTEXT_TYPE_ANY
#define AVAILABLE_SOURCE_CONTEXT BT_AUDIO_CONTEXT_TYPE_ANY

/* LC3 Codec and I2S Sample Configuration */
#define MAX_SAMPLE_RATE	      16000
#define MAX_FRAME_DURATION_US 10000
#define MAX_NUM_SAMPLES	      ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)
#define I2S_SAMPLES_NUM	      16 /* Number of samples per I2S transfer */
#define DMA_BYTES_PER_SAMPLE  4
#define RING_BUFFER_SIZE      20

/* Thread Configuration */
#define LC3_ENCODER_STACK_SIZE 8192
#define LC3_ENCODER_PRIORITY   2
#define TOTAL_BUF_NEEDED       4

/*************************************************************************************************/
/* Static Variable Declarations                                   */
/*************************************************************************************************/

/* I2S node and pin control */
#define I2S_NL DT_NODELABEL(i2s20)
PINCTRL_DT_DEFINE(I2S_NL);

/* Mouse position struct */
struct mouse_pos {
	int16_t x_val;
	int16_t y_val;
};

/* HIDS Objects */
static struct k_work hids_work;
BT_HIDS_DEF(hids_obj, INPUT_REP_BUTTONS_LEN, INPUT_REP_MOVEMENT_LEN, INPUT_REP_MEDIA_PLAYER_LEN);
K_MSGQ_DEFINE(hids_queue, sizeof(struct mouse_pos), HIDS_QUEUE_SIZE, 4);

/* I2S Driver instance and configuration */
static nrfx_i2s_t i2s_inst = NRFX_I2S_INSTANCE(20);
static nrfx_i2s_config_t cfg = {
	/* Pins are configured by pinctrl. */
	.skip_gpio_cfg = true,
	.skip_psel_cfg = true,
	.irq_priority = DT_IRQ(I2S_NL, priority),
	.mode = NRF_I2S_MODE_MASTER,
	.format = NRF_I2S_FORMAT_I2S,
	.alignment = NRF_I2S_ALIGN_LEFT,
	.ratio = NRF_I2S_RATIO_64X,
	.sample_width = NRF_I2S_SWIDTH_16BIT,
	.channels = NRF_I2S_CHANNELS_LEFT,
	.mck_setup = 0x8102000,
};

/* I2S and Audio buffers */
static uint16_t i2s_rx_buf_a[I2S_SAMPLES_NUM];
static uint16_t i2s_rx_buf_b[I2S_SAMPLES_NUM];
static int16_t send_pcm_data[MAX_NUM_SAMPLES];
RING_BUF_DECLARE(i2s_rx_ring_buf, I2S_SAMPLES_NUM * sizeof(uint16_t) * RING_BUFFER_SIZE);
NET_BUF_POOL_FIXED_DEFINE(tx_pool, CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

/* Bluetooth Connection and Stream objects */
static struct bt_conn *default_conn;
static struct bt_le_ext_adv *adv;
static struct bt_bap_stream sink_streams[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static struct audio_source {
	struct bt_bap_stream stream;
	uint16_t seq_num;
	uint16_t max_sdu;
	size_t len_to_send;
} source_streams[CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT];

/* Bluetooth LE Audio Capabilities and Configuration */
static const struct bt_audio_codec_cap lc3_codec_cap = BT_AUDIO_CODEC_CAP_LC3(
	(BT_AUDIO_CODEC_CAP_FREQ_16KHZ), BT_AUDIO_CODEC_CAP_DURATION_10,
	BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1), 40u, 120u, 1u, BT_AUDIO_CONTEXT_TYPE_ANY);
static const struct bt_bap_qos_cfg_pref qos_pref =
	BT_BAP_QOS_CFG_PREF(true, BT_GAP_LE_PHY_2M, 0x02, 10, 10000, 40000, 10000, 40000);
static struct bt_pacs_cap cap_source = {
	.codec_cap = &lc3_codec_cap,
};

/* State variables */
static uint16_t configured_sampling_freq;
static int configured_octets_per_frame;
static size_t configured_source_stream_count;

/* Advertising Data */
static uint8_t unicast_server_addata[] = {
	BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),	/* ASCS UUID */
	BT_AUDIO_UNICAST_ANNOUNCEMENT_TARGETED, /* Target Announcement */
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

/*************************************************************************************************/
/* Forward Declarations                                       */
/*************************************************************************************************/
static void i2s_fetch_thread(void *arg1, void *arg2, void *arg3);
static int i2s_mic_init(uint16_t sampling_rate);
static void disconnect_work_handler(struct k_work *work);

/* System objects */
static const struct device *gpio;
static struct k_work adv_work;
K_WORK_DEFINE(work_disconnect, disconnect_work_handler);
static K_SEM_DEFINE(lc3_encoder_sem, 0U, TOTAL_BUF_NEEDED);

/*************************************************************************************************/
/* Thread Definitions                                        */
/*************************************************************************************************/
K_THREAD_DEFINE(i2s_fetch, LC3_ENCODER_STACK_SIZE, i2s_fetch_thread, NULL, NULL, NULL,
		LC3_ENCODER_PRIORITY, 0, -1);

/*************************************************************************************************/
/* Utility Functions                                         */
/*************************************************************************************************/

void print_hex(const uint8_t *ptr, size_t len)
{
	while (len-- != 0) {
		LOG_INF("%02x", *ptr++);
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

static uint16_t get_and_incr_seq_num(const struct bt_bap_stream *stream)
{
	for (size_t i = 0U; i < configured_source_stream_count; i++) {
		if (stream == &source_streams[i].stream) {
			return source_streams[i].seq_num++;
		}
	}

	LOG_INF("Could not find endpoint from stream %p", stream);
	return 0;
}

/*************************************************************************************************/
/* I2S Audio Functions                                        */
/*************************************************************************************************/
void audio_i2s_set_next_buf(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	const nrfx_i2s_buffers_t i2s_buf = {.p_rx_buffer = rx_buf,
					    .p_tx_buffer = (uint32_t *)tx_buf,
					    .buffer_size = I2S_SAMPLES_NUM};
	nrfx_err_t ret = nrfx_i2s_next_buffers_set(&i2s_inst, &i2s_buf);

	if (ret != NRFX_SUCCESS) {
		printk("Failed to set next buffers: %x\n", ret);
	}
}

void audio_i2s_start(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	const nrfx_i2s_buffers_t i2s_buf = {.p_rx_buffer = rx_buf,
					    .p_tx_buffer = (uint32_t *)tx_buf,
					    .buffer_size = I2S_SAMPLES_NUM};

	/* Buffer size in 32-bit words */
	int ret = nrfx_i2s_start(&i2s_inst, &i2s_buf, 0);

	if (ret != NRFX_SUCCESS) {
		printk("Failed to start I2S: %d\n", ret);
	}
}

void audio_i2s_stop(void)
{
	nrfx_i2s_stop(&i2s_inst);
	nrfx_i2s_uninit(&i2s_inst);
}

static void i2s_comp_handler(nrfx_i2s_buffers_t const *released_bufs, uint32_t status)
{
	int16_t dummy_data[120] = {0};

	if (status == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {
		uint32_t ring_buf_space_bytes = ring_buf_space_get(&i2s_rx_ring_buf);

		if ((uint32_t *)released_bufs->p_rx_buffer == (uint32_t *)i2s_rx_buf_a) {
			if (ring_buf_space_bytes < (I2S_SAMPLES_NUM * DMA_BYTES_PER_SAMPLE)) {
				ring_buf_put(&i2s_rx_ring_buf, (uint8_t *)dummy_data,
					     (I2S_SAMPLES_NUM - ring_buf_space_bytes) *
						     DMA_BYTES_PER_SAMPLE);
			}
			ring_buf_put(&i2s_rx_ring_buf, (uint8_t *)i2s_rx_buf_a,
				     I2S_SAMPLES_NUM * DMA_BYTES_PER_SAMPLE);
			audio_i2s_set_next_buf(NULL, (uint32_t *)i2s_rx_buf_b);
		} else if ((uint32_t *)released_bufs->p_rx_buffer == (uint32_t *)i2s_rx_buf_b) {
			if (ring_buf_space_bytes < (I2S_SAMPLES_NUM * DMA_BYTES_PER_SAMPLE)) {
				ring_buf_put(&i2s_rx_ring_buf, (uint8_t *)dummy_data,
					     (I2S_SAMPLES_NUM - ring_buf_space_bytes) *
						     DMA_BYTES_PER_SAMPLE);
			}
			ring_buf_put(&i2s_rx_ring_buf, (uint8_t *)i2s_rx_buf_b,
				     I2S_SAMPLES_NUM * DMA_BYTES_PER_SAMPLE);
			audio_i2s_set_next_buf(NULL, (uint32_t *)i2s_rx_buf_a);
		}
	}
}

/* TODO: the sampling rate is fixed in 16KHz, so the parameter sampling_rate is not used */
static int i2s_mic_init(uint16_t sampling_rate)
{
	int ret;

	ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(I2S_NL), PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		printk("Failed to apply pinctrl state: %d\n", ret);
		return -EIO;
	}

	IRQ_CONNECT(DT_IRQN(I2S_NL), DT_IRQ(I2S_NL, priority), nrfx_isr, nrfx_i2s_20_irq_handler,
		    0);
	irq_enable(DT_IRQN(I2S_NL));

	ret = nrfx_i2s_init(&i2s_inst, &cfg, i2s_comp_handler);
	if (ret != NRFX_SUCCESS) {
		printk("Failed to initialize I2S: %x\n", ret);
		return -EIO;
	}

	return 0;
}

/*************************************************************************************************/
/* Audio Encoding/Sending                                     */
/*************************************************************************************************/

static void send_data(void)
{
	int ret;
	uint8_t lc3_encoded_buffer[CONFIG_BT_ISO_TX_MTU];
	uint16_t encoded_bytes_written;
	struct net_buf *buf;

	/*
	 * We configured the source streams to be first in `streams`, so that we can use `stream[i]`
	 * to select source streams (i.e. streams with data going to the server).
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

static void i2s_fetch_thread(void *arg1, void *arg2, void *arg3)
{
	while (true) {
		k_sem_take(&lc3_encoder_sem, K_FOREVER);
		uint32_t ring_buf_size = ring_buf_size_get(&i2s_rx_ring_buf);

		if (ring_buf_size < 16 * 2 * 10) {
			/* LOG_DBG("I2S underrun"); */
		} else {
			ring_buf_get(&i2s_rx_ring_buf, (uint8_t *)send_pcm_data, 16 * 2 * 10);
		}
		send_data();
	}
}

/*************************************************************************************************/
/* HID Functions                                             */
/*************************************************************************************************/

static void mouse_handler(struct k_work *work)
{
	struct mouse_pos pos;

	while (!k_msgq_get(&hids_queue, &pos, K_NO_WAIT)) {
		stylus_hid_mouse_movement_send(&hids_obj, pos.x_val, pos.y_val);
	}
}

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
		int err = k_msgq_put(&hids_queue, &pos, K_NO_WAIT);

		if (err) {
			printk("No space in the queue for button pressed\n");
			return;
		}
		if (k_msgq_num_used_get(&hids_queue) == 1) {
			k_work_submit(&hids_work);
		}
	}
}

/*************************************************************************************************/
/* BAP Stream Operation Callbacks                                 */
/*************************************************************************************************/

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

		memset(i2s_rx_buf_a, 0, sizeof(i2s_rx_buf_a));
		memset(i2s_rx_buf_b, 0, sizeof(i2s_rx_buf_b));
		audio_i2s_start(NULL, (uint32_t *)i2s_rx_buf_a);
		audio_i2s_set_next_buf(NULL, (uint32_t *)i2s_rx_buf_b);

		LOG_INF("I2S start trigger success");
		k_sem_give(&lc3_encoder_sem);
	}
}

static void stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
	LOG_INF("Audio Stream %p stopped with reason 0x%02X", (void *)stream, reason);

	if (stream_dir(stream) == BT_AUDIO_DIR_SOURCE) {
		dk_set_led_off(ISO_STREAM_STATUS);
		k_thread_suspend(i2s_fetch);
		audio_i2s_stop();
	}

	/*
	 * Workaround for unexpected disconnection
	 * If ISO disconnected due to timeout, disconnect the ACL connection for central to
	 * re-establish the link and stream.
	 */
	if (reason == 0x08) {
		k_work_submit(&work_disconnect);
	}
}

static void stream_enabled_cb(struct bt_bap_stream *stream)
{
	/*
	 * The unicast server is responsible for starting sink ASEs after the client has enabled
	 * them.
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
	/* Intentionally empty */
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
	.started = stream_started,
	.stopped = stream_stopped,
	.enabled = stream_enabled_cb,
	.disabled = stream_disabled_cb,
	.sent = stream_sent_cb,
};

/*************************************************************************************************/
/* BAP Unicast Server Callbacks                                    */
/*************************************************************************************************/

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

	/* We only support one QoS at the moment, reject changes */
	*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_UNSUPPORTED, BT_BAP_ASCS_REASON_NONE);
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
	LOG_INF("Release: stream %p", (void *)stream);
	if (stream_dir(stream) == BT_AUDIO_DIR_SOURCE) {
		sw_codec_lc3_enc_uninit_all();
	}
	return 0;
}

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

/*************************************************************************************************/
/* Bluetooth Connection Callbacks                                  */
/*************************************************************************************************/

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err != 0) {
		LOG_INF("Failed to connect to %s %u %s", addr, err, bt_hci_err_to_str(err));
		default_conn = NULL;
		return;
	}

	LOG_INF("Connected: %s", addr);
	default_conn = bt_conn_ref(conn);
	dk_set_led_on(ACL_LINK_STATUS);
	dk_set_led_off(ADV_STATUS);

	stylus_hid_insert_conn_object(&hids_obj, conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

	bt_conn_unref(default_conn);
	default_conn = NULL;
	configured_source_stream_count = 0U;
	dk_set_led_off(ACL_LINK_STATUS);

	stylus_hid_remove_conn_object(&hids_obj, conn);
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

/*************************************************************************************************/
/* Work Handlers                                             */
/*************************************************************************************************/

static void advertising_process(struct k_work *work)
{
	int err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);

	if (err) {
		LOG_INF("Failed to start advertising set (err %d)", err);
	}
	LOG_INF("Advertising successfully started");
	dk_set_led_on(ADV_STATUS);
}

static void disconnect_work_handler(struct k_work *work)
{
	bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

/*************************************************************************************************/
/* System Initialization                                      */
/*************************************************************************************************/

static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;
	if (!battery_level) {
		battery_level = 100U;
	}

	bt_bas_set_battery_level(battery_level);
}

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

/*************************************************************************************************/
/* main                                                 */
/*************************************************************************************************/

int main(void)
{
	int err;

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
	stylus_hid_init(&hids_obj);

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
	if (gpio_pin_get(gpio, ERASE_BOND_BTN) == 1) {
		if (IS_ENABLED(CONFIG_SETTINGS)) {
			LOG_INF("Clearing all bonds");
			err = bt_unpair(BT_ID_DEFAULT, NULL);
			if (err) {
				LOG_ERR("Failed to clear bonding: %d", err);
				return err;
			}
		}
	}

	k_work_init(&hids_work, mouse_handler);

	err = sw_codec_lc3_init(NULL, NULL, MAX_FRAME_DURATION_US);
	if (err) {
		LOG_INF("sw_codec_lc3_init failed (err %d)", err);
	}

	static struct bt_bap_unicast_server_register_param param = {
		.snk_cnt = CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT,
		.src_cnt = CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT};
	bt_bap_unicast_server_register(&param);
	bt_bap_unicast_server_register_cb(&unicast_server_cb);

	bt_pacs_cap_register(BT_AUDIO_DIR_SOURCE, &cap_source);

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
		k_sleep(K_SECONDS(5));
		bas_notify();
	}
	return 0;
}
