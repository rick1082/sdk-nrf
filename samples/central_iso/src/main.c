/*
 * Copyright (c) 2021 2021 Nordic Semiconductor ASA
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
#include <bluetooth/services/nus.h>
LOG_MODULE_REGISTER(ble);

static void start_scan(void);

static struct bt_conn *default_conn;
static struct bt_iso_chan iso_chan;

static uint32_t interval_us = 10U * USEC_PER_MSEC; /* 10 ms */

#define DEVICE_NAME "Gateway"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

#define DEVICE_NAME_PEER "Peripheral"
#define DEVICE_NAME_PEER_LEN (sizeof(DEVICE_NAME_PEER) - 1)
#define ISO_MAX_PAYLOAD 30
#define BT_LE_CONN_PARAM_MULTI BT_LE_CONN_PARAM(40, 40, 0, 400)

static int device_found(uint8_t type, const uint8_t *data, uint8_t data_len,
			const bt_addr_le_t *addr)
{
	int ret;

	if ((data_len == DEVICE_NAME_PEER_LEN) &&
	    (strncmp(DEVICE_NAME_PEER, data, DEVICE_NAME_PEER_LEN) == 0)) {
		ret = bt_le_scan_stop();
		if (ret) {
			LOG_INF("Stop scan failed: %d", ret);
		}

		ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_MULTI,
					&default_conn);
		if (ret) {
			LOG_INF("Could not init connection");
			return ret;
		}

		return 0;
	}

	return -ENOENT;
}

static void ad_parse(struct net_buf_simple *p_ad, const bt_addr_le_t *addr)
{
	while (p_ad->len > 1) {
		uint8_t len = net_buf_simple_pull_u8(p_ad);
		uint8_t type;

		/* Check for early termination */
		if (len == 0) {
			return;
		}

		if (len > p_ad->len) {
			LOG_ERR("AD malformed");
			return;
		}

		type = net_buf_simple_pull_u8(p_ad);

		if (device_found(type, p_ad->data, len - 1, addr) == 0) {
			return;
		}

		(void)net_buf_simple_pull(p_ad, len - 1);
	}
}

static void on_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			    struct net_buf_simple *p_ad)
{
	/* Direct advertising has no payload, so no need to parse */
	if (type == BT_GAP_ADV_TYPE_ADV_IND || type == BT_GAP_ADV_TYPE_SCAN_RSP ||
	    type == BT_GAP_ADV_TYPE_EXT_ADV) {
		/* Note: May lead to connection creation */
		ad_parse(p_ad, addr);
	}
}

static void start_scan(void)
{
	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, on_device_found);
	if (err) {
		LOG_INF("Scanning failed to start (err %d)", err);
		return;
	}

	LOG_INF("Scanning successfully started");
}

static void iso_connected(struct bt_iso_chan *chan)
{
	LOG_INF("ISO Channel %p connected", chan);
}

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		     struct net_buf *buf)
{
	LOG_INF("Incoming data channel %p len %u\n", chan, buf->len);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	LOG_INF("ISO Channel %p disconnected (reason 0x%02x)", chan, reason);
}

static struct bt_iso_chan_ops iso_ops = {
	.recv = iso_recv,
	.connected = iso_connected,
	.disconnected = iso_disconnected,
};

static struct bt_iso_chan_io_qos iso_rx = {
	.sdu = CONFIG_BT_ISO_TX_MTU,
	.phy = BT_GAP_LE_PHY_2M,
	.rtn = 1,
	.path = NULL,
};

static struct bt_iso_chan_qos iso_qos = {
	.rx = &iso_rx,
	.tx = NULL,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	int ret;
	struct bt_conn_info info;

	ret = bt_conn_get_info(conn, &info);

	if (info.role == BT_CONN_ROLE_CENTRAL) {
		LOG_INF("Connect to a peripheral");
		char addr[BT_ADDR_LE_STR_LEN];
		int iso_err;
		struct bt_iso_connect_param connect_param;

		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

		if (err) {
			LOG_INF("Failed to connect to %s (%u)", addr, err);

			bt_conn_unref(default_conn);
			default_conn = NULL;

			start_scan();
			return;
		}

		if (conn != default_conn) {
			return;
		}

		LOG_INF("Connected: %s", addr);

		connect_param.acl = conn;
		connect_param.iso_chan = &iso_chan;

		iso_err = bt_iso_chan_connect(&connect_param, 1);

		if (iso_err) {
			LOG_INF("Failed to connect iso (%d)", iso_err);
			return;
		}
	} else {
		LOG_INF("Connected by a central");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int ret;
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];

	ret = bt_conn_get_info(conn, &info);
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (info.role == BT_CONN_ROLE_CENTRAL) {
		if (conn != default_conn) {
			return;
		}

		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

		LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
	} else {
		LOG_INF("Disconnected from %s (reason 0x%02x)", addr, reason);
		start_scan();
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	LOG_HEXDUMP_INF(data, len, "NUS:");
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void main(void)
{
	int err;
	struct bt_iso_chan *channels[1];
	struct bt_iso_cig_param param;
	struct bt_iso_cig *cig;

	err = bt_enable(NULL);
	if (err) {
		LOG_INF("Bluetooth init failed (err %d)", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	LOG_INF("Bluetooth initialized");

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return;
	}

	iso_chan.ops = &iso_ops;
	iso_chan.qos = &iso_qos;
#if defined(CONFIG_BT_SMP)
	iso_chan.required_sec_level = BT_SECURITY_L2,
#endif /* CONFIG_BT_SMP */

	channels[0] = &iso_chan;
	param.cis_channels = channels;
	param.num_cis = ARRAY_SIZE(channels);
	param.sca = BT_GAP_SCA_UNKNOWN;
	param.packing = 0;
	param.framing = 0;
	param.latency = 10; /* ms */
	param.interval = interval_us; /* us */

	err = bt_iso_cig_create(&param, &cig);

	if (err != 0) {
		LOG_INF("Failed to create CIG (%d)", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_INF("Advertising failed to start (err %d)", err);
		return;
	}

	start_scan();
}
