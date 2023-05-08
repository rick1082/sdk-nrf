/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ble);

#define DEVICE_NAME             "Peripheral"
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct k_work_delayable iso_send_work;
static struct bt_iso_chan iso_chan;
static uint16_t seq_num;
static atomic_t iso_tx_pool_alloc;
NET_BUF_POOL_FIXED_DEFINE(tx_pool, 1, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8,
			  NULL);
#define HCI_ISO_BUF_ALLOC_PER_CHAN 2

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

	k_work_schedule(&iso_send_work, K_MSEC(10));
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

/** Print data as d_0 d_1 d_2 ... d_(n-2) d_(n-1) d_(n) to show the 3 first and 3 last octets
 *
 * Examples:
 * 01
 * 0102
 * 010203
 * 01020304
 * 0102030405
 * 010203040506
 * 010203...050607
 * 010203...060708
 * etc.
 */
static void iso_print_data(uint8_t *data, size_t data_len)
{
	/* Maximum number of octets from each end of the data */
	const uint8_t max_octets = 3;
	char data_str[35];
	size_t str_len;

	str_len = bin2hex(data, MIN(max_octets, data_len), data_str, sizeof(data_str));
	if (data_len > max_octets) {
		if (data_len > (max_octets * 2)) {
			static const char dots[] = "...";

			strcat(&data_str[str_len], dots);
			str_len += strlen(dots);
		}

		str_len += bin2hex(data + (data_len - MIN(max_octets, data_len - max_octets)),
				   MIN(max_octets, data_len - max_octets),
				   data_str + str_len,
				   sizeof(data_str) - str_len);
	}

	LOG_INF("\t %s", data_str);
}

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		struct net_buf *buf)
{
	LOG_INF("Incoming data channel %p len %u", chan, buf->len);
	iso_print_data(buf->data, buf->len);
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
	.recv		= iso_recv,
	.connected	= iso_connected,
	.disconnected	= iso_disconnected,
	.sent = iso_sent,
};

static struct bt_iso_chan_io_qos iso_tx = {
	.sdu = CONFIG_BT_ISO_TX_MTU,
	.path = NULL,
};

static struct bt_iso_chan_qos iso_qos = {
	.tx = &iso_tx,
	.rx = NULL,
};

static struct bt_iso_chan iso_chan = {
	.ops = &iso_ops,
	.qos = &iso_qos,
};

static int iso_accept(const struct bt_iso_accept_info *info,
		      struct bt_iso_chan **chan)
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

void main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_INF("Bluetooth init failed (err %d)", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	LOG_INF("Bluetooth initialized");

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
