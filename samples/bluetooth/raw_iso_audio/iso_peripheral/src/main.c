/*
 * Copyright (c) 2021-2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Standard includes */
#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/types.h>

/* Zephyr includes */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/settings/settings.h>

/* Bluetooth includes */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>

/* Driver includes */
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>

/* Nordic includes */
#include <nrfx_clock.h>
#include <nrfx_i2s.h>
#include <pcm_mix.h>

/* Audio codec includes */
#include "lc3.h"

/* Platform specific includes */
#include "nrf54l15.h"
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif

/* Audio configuration */
#define MAX_SAMPLE_RATE			48000
#define MAX_FRAME_DURATION_US		10000
#define MAX_NUM_SAMPLES			((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)

/* I2S configuration */
#define I2S_SAMPLES_NUM			48
#define BUFFER_SPACE			60
#define I2S_BUFFER_SIZE			(I2S_SAMPLES_NUM * 2 * sizeof(uint16_t))

/* Jitter buffer configuration */
#define JITTER_BUFFER_SIZE		3
#define JITTER_BUFFER_CHECK		2

/* TLV320 DAC configuration */
#define I2C_NODE			DT_NODELABEL(tlv320)
#define DEFAULT_VOLUME			-60

/* Device tree nodes */
#define I2S_NL				DT_NODELABEL(i2s20)

/* GPIO configuration */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec rst = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
static const struct device *gpio;

/* I2C configuration */
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

/* I2S configuration */
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

/* Audio buffers */
static uint16_t i2s_tx_buf_a[I2S_SAMPLES_NUM * 2] = {0};
static uint16_t i2s_tx_buf_b[I2S_SAMPLES_NUM * 2] = {0};
static uint16_t i2s_rx_buf_a[I2S_SAMPLES_NUM * 2] = {0};
static uint16_t i2s_rx_buf_b[I2S_SAMPLES_NUM * 2] = {0};

RING_BUF_DECLARE(i2s_tx_ring_buf, I2S_BUFFER_SIZE * BUFFER_SPACE);

/* LC3 decoder configuration */
static lc3_decoder_t lc3_decoder[2];
static lc3_decoder_mem_48k_t lc3_decoder_mem[2];

/* Bluetooth advertising data */
static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};

/* Packet information structure for jitter buffer */
struct recv_pkt_info {
	uint32_t sdu_ref_us;
	uint32_t recv_frame_ts_us;
	uint8_t channel;
	bool bad_frame;
	uint8_t size;
	uint8_t desired_data_size;
	uint8_t buf[CONFIG_BT_ISO_RX_MTU];
} __packed;

K_MSGQ_DEFINE(recv_pkt_msgq_l, sizeof(struct recv_pkt_info), JITTER_BUFFER_SIZE, 4);
K_WORK_DEFINE(adv_work, adv_work_handler);

/* Function declarations */
static void adv_work_handler(struct k_work *work);

/**
 * @brief Write register to TLV320 DAC via I2C
 */
static void dac_i2c_write(const struct i2c_dt_spec *dev_i2c, uint8_t reg, uint8_t value)
{
	int ret;
	uint8_t config[2] = {reg, value};

	ret = i2c_write_dt(dev_i2c, config, sizeof(config));
	if (ret != 0) {
		printk("Failed to write to I2C device address %x at reg. %x\n", dev_i2c->addr, reg);
	}
}

/**
 * @brief Setup TLV320 DAC configuration
 */
static void tlv320_setup(void)
{
	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n", dev_i2c.bus->name);
		return;
	}
	printk("I2C bus %s is ready!\n", dev_i2c.bus->name);

	/* Reset and basic configuration */
	dac_i2c_write(&dev_i2c, 0x00, 0x00);
	dac_i2c_write(&dev_i2c, 0x01, 0x01);
	k_sleep(K_MSEC(10));

	/* Clock and PLL configuration */
	dac_i2c_write(&dev_i2c, 0x04, 0x03 | (0b11 << 0));
	dac_i2c_write(&dev_i2c, 0x05, (0b001 << 4) | (0b0001 << 0));
	dac_i2c_write(&dev_i2c, 0x06, 0x05);
	dac_i2c_write(&dev_i2c, 0x07, 0x0E);
	dac_i2c_write(&dev_i2c, 0x08, 0xB0);

	dac_i2c_write(&dev_i2c, 0x05, (1 << 7) | (0b001 << 4) | (0b0001 << 0));
	k_sleep(K_MSEC(15));

	/* Audio interface configuration */
	dac_i2c_write(&dev_i2c, 0x0B, 0x87);
	dac_i2c_write(&dev_i2c, 0x0C, 0x82);
	dac_i2c_write(&dev_i2c, 0x0D, 0x00);
	dac_i2c_write(&dev_i2c, 0x0E, 0x80);

	/* Processing and power configuration */
	dac_i2c_write(&dev_i2c, 0x1B, 0x0C);
	dac_i2c_write(&dev_i2c, 0x1E, 0x84);
	dac_i2c_write(&dev_i2c, 0x1D, (0b01 << 0));
	dac_i2c_write(&dev_i2c, 0x3C, 0x01);
	dac_i2c_write(&dev_i2c, 0x74, 0x00);

	/* Output configuration */
	dac_i2c_write(&dev_i2c, 0x00, 0x01);
	dac_i2c_write(&dev_i2c, 0x1F, (0b00 << 3));
	dac_i2c_write(&dev_i2c, 0x21, (0b0111 << 3) | (0b11 << 1));
	dac_i2c_write(&dev_i2c, 0x23, 0x44);
	dac_i2c_write(&dev_i2c, 0x24, 0x80);
	dac_i2c_write(&dev_i2c, 0x25, 0x80);
	dac_i2c_write(&dev_i2c, 0x28, 0x06);
	dac_i2c_write(&dev_i2c, 0x29, 0x06);
	dac_i2c_write(&dev_i2c, 0x1F, 0xC0 | (0b00 << 3));

	k_sleep(K_MSEC(300));

	/* Volume configuration */
	dac_i2c_write(&dev_i2c, 0x00, 0x00);
	dac_i2c_write(&dev_i2c, 0x3F, 0xD4);
	dac_i2c_write(&dev_i2c, 0x41, DEFAULT_VOLUME);
	dac_i2c_write(&dev_i2c, 0x42, DEFAULT_VOLUME);
	dac_i2c_write(&dev_i2c, 0x40, 0x00);
}

/**
 * @brief Start high frequency clock
 */
static int clocks_start(void)
{
	int err, res;
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
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif

	printk("HF clock started\n");
	return 0;
}

/**
 * @brief Set next I2S buffer
 */
static void audio_i2s_set_next_buf(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	const nrfx_i2s_buffers_t i2s_buf = {
		.p_rx_buffer = rx_buf,
		.p_tx_buffer = (uint32_t *)tx_buf,
		.buffer_size = I2S_SAMPLES_NUM
	};

	nrfx_err_t ret = nrfx_i2s_next_buffers_set(&i2s_inst, &i2s_buf);
	if (ret != NRFX_SUCCESS) {
		printk("Failed to set next buffers: %x\n", ret);
	}
}

/**
 * @brief I2S completion handler
 */
static void i2s_comp_handler(nrfx_i2s_buffers_t const *released_bufs, uint32_t status)
{
	int ret;

	if (status == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {
		if ((uint16_t *)released_bufs->p_tx_buffer == i2s_tx_buf_a) {
			ret = ring_buf_get(&i2s_tx_ring_buf, (uint8_t *)i2s_tx_buf_a, I2S_BUFFER_SIZE);
			if (ret != I2S_BUFFER_SIZE) {
				memset(i2s_tx_buf_a, 0, I2S_BUFFER_SIZE);
			}
			audio_i2s_set_next_buf((const uint8_t *)i2s_tx_buf_a, (uint32_t *)i2s_rx_buf_a);
		} else if ((uint16_t *)released_bufs->p_tx_buffer == i2s_tx_buf_b) {
			ret = ring_buf_get(&i2s_tx_ring_buf, (uint8_t *)i2s_tx_buf_b, I2S_BUFFER_SIZE);
			if (ret != I2S_BUFFER_SIZE) {
				memset(i2s_tx_buf_b, 0, I2S_BUFFER_SIZE);
			}
			audio_i2s_set_next_buf((const uint8_t *)i2s_tx_buf_b, (uint32_t *)i2s_rx_buf_b);
		}
	}
}

/**
 * @brief Start I2S interface
 */
static void audio_i2s_start(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	const nrfx_i2s_buffers_t i2s_buf = {
		.p_rx_buffer = rx_buf,
		.p_tx_buffer = (uint32_t *)tx_buf,
		.buffer_size = I2S_SAMPLES_NUM
	};

	int ret = nrfx_i2s_start(&i2s_inst, &i2s_buf, 0);
	if (ret != NRFX_SUCCESS) {
		printk("Failed to start I2S: %d\n", ret);
	}
}

/**
 * @brief Initialize I2S interface
 */
static void audio_i2s_init(void)
{
	int ret;

	ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(I2S_NL), PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		printk("Failed to apply pinctrl state: %d\n", ret);
		return;
	}

	IRQ_CONNECT(DT_IRQN(I2S_NL), DT_IRQ(I2S_NL, priority), nrfx_isr,
		    nrfx_i2s_20_irq_handler, 0);
	irq_enable(DT_IRQN(I2S_NL));

	ret = nrfx_i2s_init(&i2s_inst, &cfg, i2s_comp_handler);
	if (ret != NRFX_SUCCESS) {
		printk("Failed to initialize I2S: %x\n", ret);
	}
}

/**
 * @brief Advertising work handler
 */
static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
	} else {
		printk("Advertising successfully started\n");
	}
}

/**
 * @brief Connection established callback
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s %u %s\n", addr, err, bt_hci_err_to_str(err));
		return;
	}

	printk("Connected %s\n", addr);
}

/**
 * @brief Connection lost callback
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Disconnected from %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));
	
	k_work_submit(&adv_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/**
 * @brief ISO data received callback
 */
static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		     struct net_buf *buf)
{
	static int packet_count = 0;
	
	if (++packet_count % 100 == 0) {
		/* Periodic logging for debugging */
	}

	struct recv_pkt_info pkt_info = {
		.sdu_ref_us = info->ts,
		.channel = 0,
		.size = buf->len,
		.bad_frame = (buf->len == 0 || ((info->flags & BT_ISO_FLAGS_VALID) == 0))
	};

	if (!pkt_info.bad_frame) {
		memcpy(pkt_info.buf, buf->data, buf->len);
	}

	/* Add packet to jitter buffer */
	int ret = k_msgq_put(&recv_pkt_msgq_l, &pkt_info, K_NO_WAIT);
	if (ret != 0 && ret != -ENOMSG) {
		printk("L: MsgQ full: %d\n", ret);
		struct recv_pkt_info dummy_pkt;
		k_msgq_get(&recv_pkt_msgq_l, &dummy_pkt, K_NO_WAIT);
		k_msgq_put(&recv_pkt_msgq_l, &pkt_info, K_NO_WAIT);
	}

	/* Process packets when jitter buffer has enough data */
	if (k_msgq_num_used_get(&recv_pkt_msgq_l) >= JITTER_BUFFER_CHECK) {
		struct recv_pkt_info pkt_info_l;
		k_msgq_get(&recv_pkt_msgq_l, &pkt_info_l, K_NO_WAIT);

		/* Monitor ring buffer and adjust DAC clock */
		static uint16_t prev_buf_size = 0;
		uint16_t buf_size = ring_buf_space_get(&i2s_tx_ring_buf);
		
		if (buf_size != prev_buf_size) {
			prev_buf_size = buf_size;
			int16_t buf_size_percent = buf_size * 100 / (I2S_BUFFER_SIZE * BUFFER_SPACE);
			printk("%d\n", buf_size_percent);

			/* Adjust DAC clock based on buffer fill level */
			if (buf_size_percent < 45) {
				dac_i2c_write(&dev_i2c, 0x07, 0x0E);
				dac_i2c_write(&dev_i2c, 0x08, 0xDA);
			} else if (buf_size_percent >= 40 && buf_size_percent <= 55) {
				dac_i2c_write(&dev_i2c, 0x07, 0x0E);
				dac_i2c_write(&dev_i2c, 0x08, 0xB0);
			} else {
				dac_i2c_write(&dev_i2c, 0x07, 0x0E);
				dac_i2c_write(&dev_i2c, 0x08, 0x86);
			}
		}

		/* Decode LC3 audio */
		int16_t audio_buf[2 * 480] = {0};
		int err = lc3_decode(lc3_decoder[0],
				     pkt_info_l.bad_frame ? NULL : pkt_info_l.buf,
				     pkt_info_l.size, LC3_PCM_FORMAT_S16,
				     audio_buf, 2);
		if (err != 0) {
			/* Handle decode error silently */
		}

		/* Add decoded audio to ring buffer */
		ring_buf_put(&i2s_tx_ring_buf, (uint8_t *)audio_buf, 480 * 2 * sizeof(int16_t));
	}
}

/**
 * @brief ISO channel connected callback
 */
static void iso_connected(struct bt_iso_chan *chan)
{
	const struct bt_iso_chan_path hci_path = {
		.pid = BT_ISO_DATA_PATH_HCI,
		.format = BT_HCI_CODING_FORMAT_TRANSPARENT,
	};

	printk("ISO Channel %p connected\n", chan);

	int err = bt_iso_setup_data_path(chan, BT_HCI_DATAPATH_DIR_CTLR_TO_HOST, &hci_path);
	if (err != 0) {
		printk("Failed to setup ISO RX data path: %d", err);
	}
}

/**
 * @brief ISO channel disconnected callback
 */
static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected (reason 0x%02x)\n", chan, reason);
}

static struct bt_iso_chan_ops iso_ops = {
	.recv = iso_recv,
	.connected = iso_connected,
	.disconnected = iso_disconnected,
};

static struct bt_iso_chan_io_qos iso_rx = {
	.sdu = CONFIG_BT_ISO_TX_MTU,
};

static struct bt_iso_chan_qos iso_qos = {
	.rx = &iso_rx,
	.tx = NULL,
};

static struct bt_iso_chan iso_chan = {
	.ops = &iso_ops,
	.qos = &iso_qos,
};

/**
 * @brief Accept incoming ISO connection
 */
static int iso_accept(const struct bt_iso_accept_info *info, struct bt_iso_chan **chan)
{
	printk("Incoming request from %p\n", (void *)info->acl);

	if (iso_chan.iso) {
		printk("No channels available\n");
		return -ENOMEM;
	}

	*chan = &iso_chan;
	return 0;
}

static struct bt_iso_server iso_server = {
#if defined(CONFIG_BT_SMP)
	.sec_level = BT_SECURITY_L1,
#endif
	.accept = iso_accept,
};

/**
 * @brief Main application entry point
 */
int main(void)
{
	int err;

	/* Initialize GPIO */
	gpio = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_configure_dt(&rst, GPIO_OUTPUT);

	/* Start clocks and reset DAC */
	clocks_start();
	gpio_pin_set_dt(&rst, 0);
	k_sleep(K_MSEC(1000));
	gpio_pin_set_dt(&rst, 1);
	tlv320_setup();

	/* Initialize Bluetooth */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	printk("Bluetooth initialized\n");

	/* Register ISO server */
	err = bt_iso_server_register(&iso_server);
	if (err) {
		printk("Unable to register ISO server (err %d)\n", err);
		return 0;
	}

	/* Initialize LC3 decoders */
	for (int i = 0; i < 2; i++) {
		lc3_decoder[i] = lc3_setup_decoder(10000, 16000, 48000, &lc3_decoder_mem[i]);
	}

	/* Start advertising and audio */
	k_work_submit(&adv_work);
	audio_i2s_init();
	audio_i2s_start((uint8_t *)i2s_tx_buf_a, (uint32_t *)i2s_rx_buf_a);
	audio_i2s_set_next_buf((const uint8_t *)i2s_tx_buf_b, (uint32_t *)i2s_rx_buf_b);

	printk("System initialized and ready\n");
	return 0;
}
