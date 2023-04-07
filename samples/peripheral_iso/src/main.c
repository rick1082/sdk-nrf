/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <nrfx_timer.h>
#include <nrfx_clock.h>
#include <zephyr/init.h>
#include <nrfx_dppi.h>
#include <nrfx_ipc.h>
#include <dk_buttons_and_leds.h>

LOG_MODULE_REGISTER(ble);

#define IS_LEFT_PERIPHERAL 1

#if IS_LEFT_PERIPHERAL
#define DEVICE_NAME "sync_demo_L"
#else
#define DEVICE_NAME "sync_demo_R"
#endif

#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};
static struct k_work_delayable iso_send_work;
static struct bt_iso_chan iso_chan;
static uint16_t seq_num;
static atomic_t iso_tx_pool_alloc;
NET_BUF_POOL_FIXED_DEFINE(tx_pool, 1, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8, NULL);
#define HCI_ISO_BUF_ALLOC_PER_CHAN 2
#define PRESENTATION_DELAY_US 20000
#define ISO_MAX_PAYLOAD_RX 5
#define ISO_MAX_PAYLOAD_TX 30

#define ISO_SYNC_TIMER_I2S_FRAME_START_EVT_CAPTURE_CHANNEL 0
#define ISO_SYNC_TIMER_CURR_TIME_CAPTURE_CHANNEL 1
#define ISO_SYNC_TIMER_INSTANCE_NUMBER 1

#define ISO_SYNC_TIMER_NET_APP_IPC_EVT NRF_IPC_EVENT_RECEIVE_4

const nrfx_timer_t iso_sync_timer_instance = NRFX_TIMER_INSTANCE(ISO_SYNC_TIMER_INSTANCE_NUMBER);

static uint8_t dppi_channel_timer_clear;
static uint8_t test_led_state;
static nrfx_timer_config_t cfg = { .frequency = NRF_TIMER_FREQ_1MHz,
				   .mode = NRF_TIMER_MODE_TIMER,
				   .bit_width = NRF_TIMER_BIT_WIDTH_32,
				   .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
				   .p_context = &test_led_state };

static void iso_timer_timeout(struct k_work *work)
{
	int ret;
	static uint8_t buf_data[CONFIG_BT_ISO_TX_MTU];
	struct net_buf *buf;
	static size_t len_to_send = CONFIG_BT_ISO_TX_MTU;
	static bool hci_wrn_printed;

	if (atomic_get(&iso_tx_pool_alloc) >= HCI_ISO_BUF_ALLOC_PER_CHAN) {
		if (!hci_wrn_printed) {
			LOG_WRN("HCI ISO TX overrun");
			hci_wrn_printed = true;
		}
		return;
	}

	hci_wrn_printed = false;

	buf = net_buf_alloc(&tx_pool, K_FOREVER);
	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

	net_buf_add_mem(buf, buf_data, len_to_send);

	atomic_inc(&iso_tx_pool_alloc);
	ret = bt_iso_chan_send(&iso_chan, buf, seq_num++, BT_ISO_TIMESTAMP_NONE);

	if (ret < 0) {
		LOG_INF("Failed to send ISO data (%d)", ret);
		net_buf_unref(buf);
		atomic_dec(&iso_tx_pool_alloc);
	}

	k_work_schedule(&iso_send_work, K_MSEC(5));
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_INF("Failed to connect to %s (%u)", addr, err);
		return;
	}

	LOG_INF("Connected %s", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected from %s (reason 0x%02x)", addr, reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		     struct net_buf *buf)
{
	uint32_t recv_frame_ts = nrfx_timer_capture(&iso_sync_timer_instance,
						    ISO_SYNC_TIMER_CURR_TIME_CAPTURE_CHANNEL);
	uint32_t wait_time = PRESENTATION_DELAY_US - (recv_frame_ts - info->ts);
	test_led_state = buf->data[0];
	nrfx_timer_compare(&iso_sync_timer_instance, ISO_SYNC_TIMER_CURR_TIME_CAPTURE_CHANNEL,
			   recv_frame_ts + wait_time, true);
	LOG_INF("%d sdu_ref = %d, ts = %d, diff = %d, wait = %d", test_led_state, info->ts,
		recv_frame_ts, recv_frame_ts - info->ts, wait_time);
}

static void iso_connected(struct bt_iso_chan *chan)
{
	LOG_INF("ISO Channel %p connected", chan);
	k_work_reschedule(&iso_send_work, K_MSEC(100));
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	LOG_INF("ISO Channel %p disconnected (reason 0x%02x)", chan, reason);
	k_work_cancel_delayable(&iso_send_work);
	atomic_clear(&iso_tx_pool_alloc);
}

static void iso_sent(struct bt_iso_chan *chan)
{
	atomic_dec(&iso_tx_pool_alloc);
}
static struct bt_iso_chan_ops iso_ops = {
	.recv = iso_recv,
	.connected = iso_connected,
	.disconnected = iso_disconnected,
	.sent = iso_sent,
};

static struct bt_iso_chan_io_qos iso_rx = {
	.sdu = ISO_MAX_PAYLOAD_RX,
	.path = NULL,
};

static struct bt_iso_chan_io_qos iso_tx = {
	.sdu = ISO_MAX_PAYLOAD_TX,
	.path = NULL,
};

static struct bt_iso_chan_qos iso_qos = {
	.rx = &iso_rx,
	.tx = &iso_tx,
};

static struct bt_iso_chan iso_chan = {
	.ops = &iso_ops,
	.qos = &iso_qos,
};

static int iso_accept(const struct bt_iso_accept_info *info, struct bt_iso_chan **chan)
{
	LOG_INF("Incoming request from %p", (void *)info->acl);

	if (iso_chan.iso) {
		LOG_INF("No channels available");
		return -ENOMEM;
	}

	*chan = &iso_chan;

	return 0;
}

static struct bt_iso_server iso_server = {
#if defined(CONFIG_BT_SMP)
	.sec_level = BT_SECURITY_L1,
#endif /* CONFIG_BT_SMP */
	.accept = iso_accept,
};

static void iso_sync_timer_event_handler(nrf_timer_event_t event_type, void *ctx)
{
	uint8_t led_state = *(uint8_t *)ctx;
	dk_set_led(DK_LED2, led_state);
}


static int hfclock_config_and_start(void)
{
	int ret;

	/* Use this to turn on 128 MHz clock for cpu_app */
	ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

	ret -= NRFX_ERROR_BASE_NUM;
	if (ret) {
		return ret;
	}

	nrfx_clock_hfclk_start();
	while (!nrfx_clock_hfclk_is_running()) {
	}

	return 0;
}

void main(void)
{
	int err;

	hfclock_config_and_start();
	err = bt_enable(NULL);
	if (err) {
		LOG_INF("Bluetooth init failed (err %d)", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	LOG_INF("Bluetooth initialized");
	err = dk_leds_init();
	if (err) {
		printk("LEDs init failed (err %d)\n", err);
		return;
	}
	err = bt_iso_server_register(&iso_server);
	if (err) {
		LOG_INF("Unable to register ISO server (err %d)", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_INF("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
	k_work_init_delayable(&iso_send_work, iso_timer_timeout);
}



static int iso_sync_timer_init(const struct device *unused)
{
	nrfx_err_t ret;

	ARG_UNUSED(unused);

	ret = nrfx_timer_init(&iso_sync_timer_instance, &cfg, iso_sync_timer_event_handler);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx timer init error - Return value: %d", ret);
		return ret;
	}
	IRQ_CONNECT(TIMER1_IRQn, 4, nrfx_timer_1_irq_handler, NULL, 0);
	nrfx_timer_enable(&iso_sync_timer_instance);

	/* Initialize functionality for synchronization between APP and NET core */
	ret = nrfx_dppi_channel_alloc(&dppi_channel_timer_clear);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel alloc error (timer clear) - Return value: %d", ret);
		return ret;
	}

	nrf_ipc_publish_set(NRF_IPC, ISO_SYNC_TIMER_NET_APP_IPC_EVT, dppi_channel_timer_clear);
	nrf_timer_subscribe_set(iso_sync_timer_instance.p_reg, NRF_TIMER_TASK_CLEAR,
				dppi_channel_timer_clear);
	ret = nrfx_dppi_channel_enable(dppi_channel_timer_clear);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel enable error (timer clear) - Return value: %d", ret);
		return ret;
	}

	LOG_DBG("ISO sync timer initialized");

	return 0;
}

SYS_INIT(iso_sync_timer_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);