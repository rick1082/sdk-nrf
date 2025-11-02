/*
 * Copyright (c) 2021-2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* --- Standard includes --- */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* --- Zephyr core includes --- */
#include <zephyr/autoconf.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/sys_clock.h>
#include <zephyr/types.h>
#include <zephyr/logging/log.h>

/* --- Zephyr driver includes --- */
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>

/* --- Bluetooth includes --- */
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/lc3.h>
#include <zephyr/bluetooth/audio/mcc.h>
#include <zephyr/bluetooth/audio/media_proxy.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/bluetooth/audio/vcp.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>

/* --- Nordic specific includes --- */
#include <dk_buttons_and_leds.h>
#include <nrfx_clock.h>
#include <nrfx_i2s.h>
#include <pcm_mix.h>
#include "lc3.h"
#include "nrf54l15.h"
#include "ml_main.h"
#include "main.h"

#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* --- Hardware definitions --- */
#define I2C_NODE DT_NODELABEL(tlv320)
#define I2S_NL DT_NODELABEL(i2s20)

/* --- Audio definitions --- */
#define I2S_SAMPLES_NUM 48  // samples per 1ms block
#define BUFFER_SPACE 20     // 20 buffers = 10ms total
#define MAX_SAMPLE_RATE 48000
#define MAX_FRAME_DURATION_US 10000
#define MAX_NUM_SAMPLES ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)

/* --- Button definitions --- */
#define KEY_LEFT_MASK   DK_BTN1_MSK
#define KEY_UP_MASK     DK_BTN2_MSK
#define KEY_RIGHT_MASK  DK_BTN3_MSK
#define KEY_DOWN_MASK   DK_BTN4_MSK

/* --- Bluetooth Audio Context definitions --- */
#define AVAILABLE_SINK_CONTEXT \
	(BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | \
	 BT_AUDIO_CONTEXT_TYPE_MEDIA | BT_AUDIO_CONTEXT_TYPE_GAME | \
	 BT_AUDIO_CONTEXT_TYPE_INSTRUCTIONAL)

#define AVAILABLE_SOURCE_CONTEXT BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED

/* --- GPIO declarations --- */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec rst = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
static const struct device *gpio;

/* --- I2S configuration --- */
PINCTRL_DT_DEFINE(I2S_NL);
static nrfx_i2s_t i2s_inst = NRFX_I2S_INSTANCE(20);
static nrfx_i2s_config_t cfg = {
	.skip_gpio_cfg = true,
	.skip_psel_cfg = true,
	.irq_priority = DT_IRQ(I2S_NL, priority),
	.mode = NRF_I2S_MODE_SLAVE,
	.format = NRF_I2S_FORMAT_I2S,
	.alignment = NRF_I2S_ALIGN_LEFT,
	.ratio = NRF_I2S_RATIO_64X,
	.sample_width = NRF_I2S_SWIDTH_16BIT,
	.channels = NRF_I2S_CHANNELS_STEREO,
	.mck_setup = NRF_I2S_MCK_32MDIV2,
};

/* --- I2C configuration --- */
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

/* --- Audio buffers --- */
static uint16_t i2s_tx_buf_a[I2S_SAMPLES_NUM * 2] = {0}; // 2 channels, 16 bits each
static uint16_t i2s_tx_buf_b[I2S_SAMPLES_NUM * 2] = {0};
static uint16_t i2s_rx_buf_a[I2S_SAMPLES_NUM * 2] = {0};
static uint16_t i2s_rx_buf_b[I2S_SAMPLES_NUM * 2] = {0};
RING_BUF_DECLARE(i2s_tx_ring_buf, I2S_SAMPLES_NUM * 2 * sizeof(uint16_t) * BUFFER_SPACE);

/* --- Bluetooth Audio variables --- */
NET_BUF_POOL_FIXED_DEFINE(tx_pool, CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static const struct bt_audio_codec_cap lc3_codec_cap = BT_AUDIO_CODEC_CAP_LC3(
	BT_AUDIO_CODEC_CAP_FREQ_48KHZ, BT_AUDIO_CODEC_CAP_DURATION_10,
	BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(2), 80u, 240u, 1u, AVAILABLE_SINK_CONTEXT);

static const struct bt_bap_qos_cfg_pref qos_pref =
	BT_BAP_QOS_CFG_PREF(true, BT_GAP_LE_PHY_2M, 10, 10, 10000, 40000, 10000, 40000);

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

/* --- LC3 decoder variables --- */
static lc3_decoder_t lc3_decoder[2];
static lc3_decoder_mem_48k_t lc3_decoder_mem[2];
static int frames_per_sdu;

/* --- Volume Control variables --- */
static struct bt_vcp_included vcp_included;

/* --- Advertising variables --- */
static struct bt_le_ext_adv *adv;
static struct k_work adv_work;

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

/* --- Function prototypes --- */
int mcp_send_cmd(uint8_t mcp_opcode);
static int pdm_mic_init(uint16_t sampling_rate);

/* --- I2C helper functions --- */
void dac_i2c_write(const struct i2c_dt_spec *dev_i2c, uint8_t reg, uint8_t value)
{
	int ret;
	uint8_t config[2] = {reg, value};

	ret = i2c_write_dt(dev_i2c, config, sizeof(config));
	if (ret != 0) {
		printk("Failed to write to I2C device address %x at reg. %x\n", dev_i2c->addr, reg);
	} else {
		// printk("I2C device address %x at reg. %x written successfully\n", dev_i2c->addr,
		//        reg);
	}
}

void tlv320_setup(void)
{

	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n", dev_i2c.bus->name);
		return;
	} else {
		printk("I2C bus %s is ready!\n", dev_i2c.bus->name);
	}

	dac_i2c_write(&dev_i2c, 0x00, 0x00);

	dac_i2c_write(&dev_i2c, 0x01, 0x01);
	k_sleep(K_MSEC(10));

	dac_i2c_write(&dev_i2c, 0x04, 0x03 | (0b11 << 0));
	dac_i2c_write(&dev_i2c, 0x05, (0b001 << 4) | (0b0001 << 0));
	dac_i2c_write(&dev_i2c, 0x06, 0x05);
	dac_i2c_write(&dev_i2c, 0x07, 0x0E);
	dac_i2c_write(&dev_i2c, 0x08, 0xB0);

	dac_i2c_write(&dev_i2c, 0x05, (1 << 7) | (0b001 << 4) | (0b0001 << 0));
	k_sleep(K_MSEC(15));

	dac_i2c_write(&dev_i2c, 0x0B, 0x87);
	dac_i2c_write(&dev_i2c, 0x0C, 0x82);
	dac_i2c_write(&dev_i2c, 0x0D, 0x00);
	dac_i2c_write(&dev_i2c, 0x0E, 0x80);

	dac_i2c_write(&dev_i2c, 0x1B, 0x0C);
	dac_i2c_write(&dev_i2c, 0x1E, 0x84);
	dac_i2c_write(&dev_i2c, 0x1D, (0b01 << 0));
	dac_i2c_write(&dev_i2c, 0x3C, 0x01);
	dac_i2c_write(&dev_i2c, 0x74, 0x00);

	dac_i2c_write(&dev_i2c, 0x00, 0x01);
	dac_i2c_write(&dev_i2c, 0x1F, (0b00 << 3));
	dac_i2c_write(&dev_i2c, 0x21, (0b0111 << 3) | (0b11 << 1));
	dac_i2c_write(&dev_i2c, 0x23, 0x44);
	dac_i2c_write(&dev_i2c, 0x24, 0x80);
	dac_i2c_write(&dev_i2c, 0x25, 0x80);
	dac_i2c_write(&dev_i2c, 0x28, 0x06);
	dac_i2c_write(&dev_i2c, 0x29, 0x06);
	dac_i2c_write(&dev_i2c, 0x1F, 0xC0 | (0b00 << 3));
	// dac_i2c_write(&dev_i2c, 0x20, 0x80);

	k_sleep(K_MSEC(300));

	dac_i2c_write(&dev_i2c, 0x00, 0x00);
	dac_i2c_write(&dev_i2c, 0x3F, 0xD4);
	dac_i2c_write(&dev_i2c, 0x41, -60);
	dac_i2c_write(&dev_i2c, 0x42, -60);
	dac_i2c_write(&dev_i2c, 0x40, 0x00);
	dac_i2c_write(&dev_i2c, 0x00, 0x00);
}

/* --- I2S functions --- */
void audio_i2s_set_next_buf(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	const nrfx_i2s_buffers_t i2s_buf = {.p_rx_buffer = rx_buf,
					    .p_tx_buffer = (uint32_t *)tx_buf,
					    .buffer_size = I2S_SAMPLES_NUM};

	nrfx_err_t ret;

	ret = nrfx_i2s_next_buffers_set(&i2s_inst, &i2s_buf);
	if (ret != NRFX_SUCCESS) {
		printk("Failed to set next buffers: %x\n", ret);
	}
}

static void i2s_comp_handler(nrfx_i2s_buffers_t const *released_bufs, uint32_t status)
{
	int ret;

	if (status == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {
		if ((uint16_t *)released_bufs->p_tx_buffer == i2s_tx_buf_a) {
			ret = ring_buf_get(&i2s_tx_ring_buf, (uint8_t *)i2s_tx_buf_a,
					   I2S_SAMPLES_NUM * 2 * sizeof(uint16_t));
			if (ret != 192) {
				memset(i2s_tx_buf_a, 0, 192);
			}
			audio_i2s_set_next_buf((const uint8_t *)i2s_tx_buf_a,
					       (uint32_t *)i2s_rx_buf_a);
		} else if ((uint16_t *)released_bufs->p_tx_buffer == i2s_tx_buf_b) {
			ret = ring_buf_get(&i2s_tx_ring_buf, (uint8_t *)i2s_tx_buf_b,
					   I2S_SAMPLES_NUM * 2 * sizeof(uint16_t));
			if (ret != 192) {
				memset(i2s_tx_buf_b, 0, 192);
			}
			audio_i2s_set_next_buf((const uint8_t *)i2s_tx_buf_b,
					       (uint32_t *)i2s_rx_buf_b);
		}
	}
}

void audio_i2s_start(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	const nrfx_i2s_buffers_t i2s_buf = {.p_rx_buffer = rx_buf,
					    .p_tx_buffer = (uint32_t *)tx_buf,
					    .buffer_size = I2S_SAMPLES_NUM};

	int ret;

	/* Buffer size in 32-bit words */
	ret = nrfx_i2s_start(&i2s_inst, &i2s_buf, 0);
	if (ret != NRFX_SUCCESS) {
		printk("Failed to start I2S: %d\n", ret);
	}
}

void audio_i2s_init(void)
{
	int ret;

	ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(I2S_NL), PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		printk("Failed to apply pinctrl state: %d\n", ret);
		return;
	}

	IRQ_CONNECT(DT_IRQN(I2S_NL), DT_IRQ(I2S_NL, priority), nrfx_isr, nrfx_i2s_20_irq_handler,
		    0);
	irq_enable(DT_IRQN(I2S_NL));

	ret = nrfx_i2s_init(&i2s_inst, &cfg, i2s_comp_handler);
	if (ret != NRFX_SUCCESS) {
		printk("Failed to initialize I2S: %x\n", ret);
		return;
	}
}

/* --- Clock management --- */
static int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		printk("Unable to get the Clock manager\n");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		printk("Clock request failed: %d\n", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			printk("Clock could not be started: %d\n", res);
			return res;
		}
	} while (err);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	printk("HF clock started\n");
	return 0;
}

/* --- Utility functions --- */
void print_hex(const uint8_t *ptr, size_t len)
{
	while (len-- != 0) {
		printk("%02x", *ptr++);
	}
}

static bool print_cb(struct bt_data *data, void *user_data)
{
	const char *str = (const char *)user_data;

	//LOG_INF("%s: type 0x%02x value_len %u", str, data->type, data->data_len);
	//LOG_HEXDUMP_INF(data->data, data->data_len, "value:");

	return true;
}

static void print_codec_cfg(const struct bt_audio_codec_cfg *codec_cfg)
{
	//LOG_INF("codec_cfg 0x%02x cid 0x%04x vid 0x%04x count %u", codec_cfg->id, codec_cfg->cid,
	//	codec_cfg->vid, codec_cfg->data_len);

	if (codec_cfg->id == BT_HCI_CODING_FORMAT_LC3) {
		enum bt_audio_location chan_allocation;
		int ret;

		/* LC3 uses the generic LTV format - other codecs might do as well */

		bt_audio_data_parse(codec_cfg->data, codec_cfg->data_len, print_cb, "data");

		ret = bt_audio_codec_cfg_get_freq(codec_cfg);
		if (ret > 0) {
			LOG_INF("  Frequency: %d Hz", bt_audio_codec_cfg_freq_to_freq_hz(ret));
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

		LOG_INF("  Octets per frame: %d (negative means value not pressent)",
			bt_audio_codec_cfg_get_octets_per_frame(codec_cfg));
		LOG_INF("  Frames per SDU: %d",
			bt_audio_codec_cfg_get_frame_blocks_per_sdu(codec_cfg, true));
	} else {
		print_hex(codec_cfg->data, codec_cfg->data_len);
	}

	bt_audio_data_parse(codec_cfg->meta, codec_cfg->meta_len, print_cb, "meta");
}

static void print_qos(const struct bt_bap_qos_cfg *qos)
{
	LOG_INF("QoS: interval %u framing 0x%02x phy 0x%02x sdu %u "
		"rtn %u latency %u pd %u",
		qos->interval, qos->framing, qos->phy, qos->sdu, qos->rtn, qos->latency, qos->pd);
}

/* --- Volume Control callbacks --- */
static void vcs_state_cb(struct bt_conn *conn, int err, uint8_t volume, uint8_t mute)
{
	if (err) {
		printk("VCS state get failed (%d)\n", err);
	} else {
		printk("VCS volume %u, mute %u\n", volume, mute);
		dac_i2c_write(&dev_i2c, 0x41, (int8_t)(volume-127));
		dac_i2c_write(&dev_i2c, 0x42, (int8_t)(volume-127));
	}
}

static void vcs_flags_cb(struct bt_conn *conn, int err, uint8_t flags)
{
	if (err) {
		printk("VCS flags get failed (%d)\n", err);
	} else {
		printk("VCS flags 0x%02X\n", flags);
	}
}

static struct bt_vcp_vol_rend_cb vcp_cbs = {
	.state = vcs_state_cb,
	.flags = vcs_flags_cb,
};

static int vcp_vol_renderer_init(void)
{
	int err;
	struct bt_vcp_vol_rend_register_param vcp_register_param;

	memset(&vcp_register_param, 0, sizeof(vcp_register_param));

	vcp_register_param.step = 10;
	vcp_register_param.mute = BT_VCP_STATE_UNMUTED;
	vcp_register_param.volume = 100;
	vcp_register_param.cb = &vcp_cbs;

	err = bt_vcp_vol_rend_register(&vcp_register_param);
	if (err) {
		return err;
	}

	err = bt_vcp_vol_rend_included_get(&vcp_included);
	if (err != 0) {
		return err;
	}

	return 0;
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

	__ASSERT(false, "Invalid stream %p", (void *)stream);
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
		configured_source_stream_count++;
	}

	*pref = qos_pref;

	/* Nothing to free as static memory is used */
	for (int i = 0; i < 2; i++) {
		lc3_decoder[i] = NULL;
	}

	return 0;
}

static int lc3_reconfig(struct bt_bap_stream *stream, enum bt_audio_dir dir,
			const struct bt_audio_codec_cfg *codec_cfg,
			struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("ASE Codec Reconfig: stream %p", (void *)stream);

	print_codec_cfg(codec_cfg);

	/* Nothing to free as static memory is used */
	for (int i = 0; i < 2; i++) {
		lc3_decoder[i] = NULL;
	}

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

	int frame_duration_us;
	int freq;
	int ret;

	ret = bt_audio_codec_cfg_get_freq(stream->codec_cfg);
	if (ret > 0) {
		freq = bt_audio_codec_cfg_freq_to_freq_hz(ret);
	} else {
		LOG_INF("Error: Codec frequency not set, cannot start codec.");
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
				       BT_BAP_ASCS_REASON_CODEC_DATA);
		return ret;
	}

	ret = bt_audio_codec_cfg_get_frame_dur(stream->codec_cfg);
	if (ret > 0) {
		frame_duration_us = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);
	} else {
		LOG_INF("Error: Frame duration not set, cannot start codec.");
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
				       BT_BAP_ASCS_REASON_CODEC_DATA);
		return ret;
	}

	frames_per_sdu = bt_audio_codec_cfg_get_frame_blocks_per_sdu(stream->codec_cfg, true);

	for (int i = 0; i < 2; i++) {
		lc3_decoder[i] = lc3_setup_decoder(frame_duration_us, freq, 0, /* No resampling */
						   &lc3_decoder_mem[i]);

		if (lc3_decoder[i] == NULL) {
			LOG_INF("ERROR: Failed to setup LC3 encoder - wrong parameters?");
			*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
					       BT_BAP_ASCS_REASON_CODEC_DATA);
			return -1;
		}
	}

	return 0;
}

static int lc3_start(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	LOG_INF("Start: stream %p", (void *)stream);

	for (size_t i = 0U; i < configured_source_stream_count; i++) {
		if (stream == &source_streams[i].stream) {
			source_streams[i].seq_num = 0U;
			source_streams[i].len_to_send = 0U;
			break;
		}
	}

	if (configured_source_stream_count > 0 && !k_work_delayable_is_pending(&audio_send_work)) {

		/* Start send timer */
		//k_work_schedule(&audio_send_work, K_MSEC(0));
	}

	return 0;
}

static bool data_func_cb(struct bt_data *data, void *user_data)
{
	struct bt_bap_ascs_rsp *rsp = (struct bt_bap_ascs_rsp *)user_data;

	if (!BT_AUDIO_METADATA_TYPE_IS_KNOWN(data->type)) {
		LOG_INF("Invalid metadata type %u or length %u", data->type, data->data_len);
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_METADATA_REJECTED, data->type);

		return -EINVAL;
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
static void stream_recv_lc3_codec(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info,
				  struct net_buf *buf)
{
	bool valid_data = (info->flags & BT_ISO_FLAGS_VALID) != 0;
	const int octets_per_frame = buf->len / frames_per_sdu;

	for (int i = 0; i < 2; i++) {
		if (lc3_decoder[i] == NULL) {
			LOG_INF("LC3 decoder not setup, cannot decode data.");
			return;
		}
	}

	if (buf->len == 0) {
		//LOG_INF("Received empty buffer");
		valid_data = false;
	}

	if (!valid_data) {
		//LOG_INF("Bad packet: 0x%02X", info->flags);
	}

	int16_t audio_buf_test[2 * 480];
	// LOG_INF("RX stream %p len %u", stream, buf->len);
	uint16_t buf_size;
	static uint16_t prev_buf_size = 0;

	buf_size = ring_buf_space_get(&i2s_tx_ring_buf);
	if (buf_size != prev_buf_size) {
		prev_buf_size = buf_size;
		//LOG_INF("I2S TX ring buffer space: %d bytes", buf_size);
		int16_t buf_size_percent = buf_size * 100 / (I2S_SAMPLES_NUM * 2 * sizeof(uint16_t) * BUFFER_SPACE);
		//LOG_INF("FIFO state%d", buf_size_percent);
		if (buf_size_percent < 45) {
			dac_i2c_write(&dev_i2c, 0x07, 0x0E); // D[13:8] for D=3760
		    dac_i2c_write(&dev_i2c, 0x08, 0xDA); // D[7:0] for D=3760
		} else if (buf_size_percent >= 40 && buf_size_percent <= 55) {
			dac_i2c_write(&dev_i2c, 0x07, 0x0E); // D[13:8] for D=3760
			dac_i2c_write(&dev_i2c, 0x08, 0xB0); // D[7:0] for D=3760
		} else {
			dac_i2c_write(&dev_i2c, 0x07, 0x0E); // D[13:8] for D=3760
		    dac_i2c_write(&dev_i2c, 0x08, 0x86); // D[7:0] for D=3760
		}
	}

	for (int i = 0; i < frames_per_sdu; i++) {
		for (int j = 0; j < 2; j++) {
			const int err = lc3_decode(
				lc3_decoder[j],
				valid_data ? net_buf_pull_mem(buf, octets_per_frame / 2) : NULL,
				octets_per_frame / 2, LC3_PCM_FORMAT_S16, audio_buf_test + j, 2);
			if (err == 1) {
				LOG_DBG("[%d]: Decoder performed PLC", i);
			} else if (err < 0) {
				LOG_INF("[%d]: Decoder failed - wrong parameters?: %d", i, err);
			}
		}
	}
	ring_buf_put(&i2s_tx_ring_buf, (uint8_t *)audio_buf_test, 480 * 2 * sizeof(int16_t));
}

static void stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
	LOG_INF("Audio Stream %p stopped with reason 0x%02X", (void *)stream, reason);

	/* Stop send timer */
	//k_work_cancel_delayable(&audio_send_work);
}

static void stream_started(struct bt_bap_stream *stream)
{
	LOG_INF("Audio Stream %p started", (void *)stream);

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

static struct bt_bap_stream_ops stream_ops = {
	.recv = stream_recv_lc3_codec,
	.stopped = stream_stopped,
	.started = stream_started,
	.enabled = stream_enabled_cb,
};

/* --- Advertising functions --- */
static void advertising_process(struct k_work *work)
{
	int err;
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		LOG_INF("Failed to start advertising set (err %d)", err);
	}
	LOG_INF("Advertising successfully started");
}

/* --- Media Control functions --- */
static void mcc_discover_mcs_cb(struct bt_conn *conn, int err)
{
	LOG_WRN("MCP: MCS discovery complete callback");
	if (err) {
		printk("MCP: Discovery of MCS failed (%d)\n", err);
	} else {
		printk("MCP: Discovered MCS\n");
	}
}

static void mcc_send_command_cb(struct bt_conn *conn, int err, const struct mpl_cmd *cmd)
{
	if (err) {
		printk("MCP: Command send failed (%d) - opcode: %u, param: %d\n",
			err, cmd->opcode, cmd->param);
	} else {
		printk("MCP: Successfully sent command (%d) - opcode: %u, param: %d\n",
			err, cmd->opcode, cmd->param);
	}
}

static struct bt_mcc_cb mcc_cb = {
	.discover_mcs = mcc_discover_mcs_cb,
	.send_cmd = mcc_send_command_cb,
};

int mcp_ctlr_init(struct bt_conn *conn)
{
	int err;

	default_conn = bt_conn_ref(conn);

	err = bt_mcc_init(&mcc_cb);
	if (err != 0) {
		return err;
	}

	err = bt_mcc_discover_mcs(default_conn, true);

	return err;
}

int mcp_send_cmd(uint8_t mcp_opcode)
{
	int err;
	struct mpl_cmd cmd;

	cmd.opcode = mcp_opcode;
	cmd.use_param = false;

	if (default_conn == NULL) {
		printk("MCP: No connection\n");
		return -EINVAL;
	}

	err = bt_mcc_send_cmd(default_conn, &cmd);
	if (err != 0) {
		printk("MCP: Command failed: %d\n", err);
	}

	return err;
}

/* --- Connection callbacks --- */
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

	configured_source_stream_count = 0;
	//k_work_cancel_delayable(&audio_send_work);

	k_work_submit(&adv_work);
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	int ret;

	if (err) {
		LOG_WRN("Security failed: level %d err %d %s", level, err,
			bt_security_err_to_str(err));
		ret = bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
		if (ret) {
			LOG_WRN("Failed to disconnect %d", ret);
		}
	} else {
		LOG_INF("Security changed: level %d", level);
		mcp_ctlr_init(conn);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed_cb,
};

/* --- PACS capabilities --- */
static struct bt_pacs_cap cap_sink = {
	.codec_cap = &lc3_codec_cap,
};

static struct bt_pacs_cap cap_source = {
	.codec_cap = &lc3_codec_cap,
};

/* --- PACS setup functions --- */
static int set_location(void)
{
	int err;

	if (IS_ENABLED(CONFIG_BT_PAC_SNK_LOC)) {
		err = bt_pacs_set_location(BT_AUDIO_DIR_SINK, (BT_AUDIO_LOCATION_FRONT_LEFT |
							       BT_AUDIO_LOCATION_FRONT_RIGHT));
		if (err != 0) {
			LOG_INF("Failed to set sink location (err %d)", err);
			return err;
		}
	}

	if (IS_ENABLED(CONFIG_BT_PAC_SRC_LOC)) {
		err = bt_pacs_set_location(BT_AUDIO_DIR_SOURCE, (BT_AUDIO_LOCATION_FRONT_LEFT |
								 BT_AUDIO_LOCATION_FRONT_RIGHT));
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

/* --- Button handling --- */
static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (buttons & DK_BTN1_MSK) {
		LOG_INF("Button 1 pressed");
		mcp_send_cmd(MEDIA_PROXY_OP_PLAY);
	}
	if (buttons & DK_BTN2_MSK) {
		LOG_INF("Button 2 pressed");
		mcp_send_cmd(MEDIA_PROXY_OP_PAUSE);
	}
	if (buttons & DK_BTN3_MSK) {
		LOG_INF("Button 3 pressed");
	}
	if (buttons & DK_BTN4_MSK) {
		LOG_INF("Button 4 pressed");
	}

}
#include "hal/nrf_pdm.h"
#include <zephyr/audio/dmic.h>


#define GAIN_DEFAULT	      0x80 //0x50
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
#define MAX_BLOCK_SIZE BLOCK_SIZE(16000, 2)
#define BLOCK_COUNT    8
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 8);
static const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));

#define LC3_ENCODER_STACK_SIZE 2048
#define LC3_ENCODER_PRIORITY   4
static void dmic_fetch_thread(void *arg1, void *arg2, void *arg3);
K_THREAD_DEFINE(dmic_fetch, LC3_ENCODER_STACK_SIZE, dmic_fetch_thread, NULL, NULL, NULL,
		LC3_ENCODER_PRIORITY, 0, -1);

static void dmic_fetch_thread(void *arg1, void *arg2, void *arg3)
{
	int ret, err;
	void *buffer;
	uint32_t size;
	static int i = 0;
	uint8_t ml_buffer[320];
	err = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (err < 0) {
		LOG_INF("DMIC start trigger failed: %d", err);
	} else {
		LOG_INF("DMIC start trigger success");
	}
	
	while (true) {
		ret = dmic_read(dmic_dev, 0, &buffer, &size, 20);
		if (ret < 0) {
			LOG_INF("DMIC read failed: %d", ret);
			k_mem_slab_free(&mem_slab, buffer);
			dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
			//dmic_trigger(dmic_dev, DMIC_TRIGGER_RESET);
			k_yield();
			k_sleep(K_MSEC(1000));
			dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
		} else {
			memcpy(ml_buffer, buffer,size);
			k_mem_slab_free(&mem_slab, buffer);
			ml_process(ml_buffer, size);
		}
	}
}

static int pdm_mic_init(uint16_t sampling_rate)
{
	int err;
	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io =
			{
				/* These fields can be used to limit the PDM clock
				 * configurations that the driver is allowed to use
				 * to those supported by the microphone.
				 */
				.min_pdm_clk_freq = 1000000,
				.max_pdm_clk_freq = 3250000,
				.min_pdm_clk_dc = 40,
				.max_pdm_clk_dc = 60,
			},
		.streams = &stream,
		.channel =
			{
				.req_num_streams = 1,
			},
	};

	err = device_is_ready(dmic_dev);
	if (err < 0) {
		LOG_INF("DMIC device is not ready: %d", err);
		return err;
	}

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = sampling_rate;
	cfg.streams[0].block_size = BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	err = dmic_configure(dmic_dev, &cfg);
	if (err < 0) {
		LOG_INF("Failed to configure the driver: %d", err);
		return err;
	}

	nrf_pdm_gain_set(NRF_PDM20_S, GAIN_DEFAULT, GAIN_DEFAULT);
	k_thread_start(dmic_fetch);

	return 0;
}

int main(void)
{
	int err;

	/* Hardware initialization */
	gpio = DEVICE_DT_GET(DT_NODELABEL(gpio1));
	gpio_pin_configure(gpio, 13, GPIO_INPUT | GPIO_PULL_UP);
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_configure_dt(&rst, GPIO_OUTPUT);

	clocks_start();
	gpio_pin_set_dt(&rst, 0);
	k_sleep(K_MSEC(1000));
	gpio_pin_set_dt(&rst, 1);

	/* Audio hardware setup */
	tlv320_setup();
	audio_i2s_init();
	audio_i2s_start((uint8_t *)i2s_tx_buf_a, (uint32_t *)i2s_rx_buf_a);
	audio_i2s_set_next_buf((const uint8_t *)i2s_tx_buf_b, (uint32_t *)i2s_rx_buf_b);

	/* Bluetooth initialization */
	err = bt_enable(NULL);
	if (err != 0) {
		LOG_INF("Bluetooth init failed (err %d)", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* Check for bond clearing */
	
	err = gpio_pin_get(gpio, 13);
	if (err == 0) {
		if (IS_ENABLED(CONFIG_SETTINGS)) {
			LOG_WRN("Clearing all bonds");
			err = bt_unpair(BT_ID_DEFAULT, NULL);
			if (err) {
				LOG_ERR("Failed to clear bonding: %d", err);
				return err;
			}
		}
	}

	LOG_INF("Bluetooth initialized");

	/* BAP server setup */
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

	/* Volume control setup */
	vcp_vol_renderer_init();

	/* Advertising setup */
	err = bt_le_ext_adv_create(BT_BAP_ADV_PARAM_CONN_QUICK, NULL, &adv);
	if (err) {
		LOG_INF("Failed to create advertising set (err %d)", err);
		return 0;
	}

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_INF("Failed to set advertising data (err %d)", err);
		return 0;
	}

	ml_init();	
	pdm_mic_init(16000);

	k_work_init(&adv_work, advertising_process);
	k_work_submit(&adv_work);

	/* Main loop */
	while (true) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
