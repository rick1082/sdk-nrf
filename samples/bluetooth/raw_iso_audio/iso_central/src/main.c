/*
 * Copyright (c) 2021-2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/gap.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>

static void start_scan(void);

static struct bt_conn *peripheral_conn[2];
static struct k_work_delayable iso_send_work;
static struct bt_iso_chan iso_chan[2];
static uint16_t seq_num;
static uint16_t latency_ms = 10U; /* 10ms */
static uint32_t interval_us = 10U * USEC_PER_MSEC; /* 10 ms */
NET_BUF_POOL_FIXED_DEFINE(tx_pool, 1, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);


static int conn_num_get(struct bt_conn *conn)
{
	for (size_t i = 0; i < ARRAY_SIZE(peripheral_conn); i++) {
		if (peripheral_conn[i] == conn) {
			return i;
		}
	}

	return -1;
}

static bool all_conn_connected(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(peripheral_conn); i++) {
		if (peripheral_conn[i] == NULL) {
			return false;
		}
	}

	return true;
}

static int conn_empty_slot_get(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(peripheral_conn); i++) {
		if (peripheral_conn[i] == NULL) {
			return i;
		}
	}

	return -1;
}

static bool all_iso_connected(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(iso_chan); i++) {
		if (iso_chan[i].state != BT_ISO_STATE_CONNECTED) {
			return false;
		}
	}

	return true;
}

static bool all_iso_disconnected(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(iso_chan); i++) {
		if (iso_chan[i].state != BT_ISO_STATE_DISCONNECTED) {
			return false;
		}
	}

	return true;
}	

/**
 * @brief Send ISO data on timeout
 *
 * This will send an increasing amount of ISO data, starting from 1 octet.
 *
 * First iteration : 0x00
 * Second iteration: 0x00 0x01
 * Third iteration : 0x00 0x01 0x02
 *
 * And so on, until it wraps around the configured ISO TX MTU (CONFIG_BT_ISO_TX_MTU)
 *
 * @param work Pointer to the work structure
 */
static void iso_timer_timeout(struct k_work *work)
{
	printk("ISO timer timeout\n");
#if 0
	int ret;
	static uint8_t buf_data[CONFIG_BT_ISO_TX_MTU];
	static bool data_initialized;
	struct net_buf *buf;
	static size_t len_to_send = 1;

	if (!data_initialized) {
		for (int i = 0; i < ARRAY_SIZE(buf_data); i++) {
			buf_data[i] = (uint8_t)i;
		}

		data_initialized = true;
	}

	buf = net_buf_alloc(&tx_pool, K_NO_WAIT);
	if (buf != NULL) {
		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

		net_buf_add_mem(buf, buf_data, len_to_send);

		ret = bt_iso_chan_send(&iso_chan, buf, seq_num);

		if (ret < 0) {
			printk("Failed to send ISO data (%d)\n", ret);
			net_buf_unref(buf);
		}

		len_to_send++;
		if (len_to_send > ARRAY_SIZE(buf_data)) {
			len_to_send = 1;
		}
	} else {
		printk("Failed to allocate buffer, retrying in next interval (%u us)\n",
		       interval_us);
	}

	/* Sequence number shall be incremented for each SDU interval */
	seq_num++;
#endif
	//k_work_schedule(&iso_send_work, K_USEC(interval_us));
	k_work_schedule(&iso_send_work, K_MSEC(1000));
}
static const char srch_name[] = "Zephyr ISO server";
static bool device_name_check(struct bt_data *data, void *user_data)
{
	int ret;
	bt_addr_le_t *addr = user_data;
	char addr_string[BT_ADDR_LE_STR_LEN];

	const struct bt_le_conn_param *conn_param =
		BT_LE_CONN_PARAM(BT_GAP_MS_TO_CONN_INTERVAL(60), BT_GAP_MS_TO_CONN_INTERVAL(60), 0,
				 BT_GAP_MS_TO_CONN_TIMEOUT(4000));
	/* We only care about LTVs with name */
	if (data->type == BT_DATA_NAME_COMPLETE || data->type == BT_DATA_NAME_SHORTENED) {
		size_t srch_name_size = strlen(srch_name);
		//printk("Device found: %s %d\n", data->data, (memcmp(srch_name, data->data, srch_name_size-1)));
		if ((memcmp(srch_name, data->data, srch_name_size-1) == 0)) {

			//printk("Device found: %s\n", srch_name);

			ret = bt_le_scan_stop();
			if (ret) {
				printk("Stop scan failed: %d\n", ret);
			}

			bt_addr_le_to_str(addr, addr_string, BT_ADDR_LE_STR_LEN);

			printk("Creating connection to device: %s\n", addr_string);

			ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, conn_param, &peripheral_conn[conn_empty_slot_get()]);
			if (ret) {
				printk("Create conn to %s failed (%u)\n", addr_string, ret);
				start_scan();
			}

			return false;
		}
	}

	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{

	if (all_conn_connected()) {
		/* Already connected */
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	bt_data_parse(ad, device_name_check, (void *)addr);

}

static void start_scan(void)
{
	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static void iso_connected(struct bt_iso_chan *chan)
{
	const struct bt_iso_chan_path hci_path = {
		.pid = BT_ISO_DATA_PATH_HCI,
		.format = BT_HCI_CODING_FORMAT_TRANSPARENT,
	};
	int err;

	printk("ISO Channel %p connected\n", chan);

	seq_num = 0U;

	err = bt_iso_setup_data_path(chan, BT_HCI_DATAPATH_DIR_HOST_TO_CTLR, &hci_path);
	if (err != 0) {
		printk("Failed to setup ISO TX data path: %d\n", err);
	} else {
		/* Start send timer */
		if (iso_chan[0].state != BT_ISO_STATE_CONNECTED ||
		    iso_chan[1].state != BT_ISO_STATE_CONNECTED) {
			printk("One ISO channel connected, starting ISO send timer\n");
			k_work_schedule(&iso_send_work, K_NO_WAIT);
		}
		
	}
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	int err;

	printk("ISO Channel %p disconnected (reason 0x%02x)\n", chan, reason);
	

	err = bt_iso_remove_data_path(chan, BT_HCI_DATAPATH_DIR_HOST_TO_CTLR);
	if (err != 0) {
		printk("Failed to setup ISO TX data path: %d\n", err);
	}

	if (all_iso_disconnected()) {
		printk("All ISO channels disconnected, stop iso_send_work\n");
		k_work_cancel_delayable(&iso_send_work);
	}
}

static void iso_sent(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p sent\n", chan);
}

static struct bt_iso_chan_ops iso_ops = {
	.connected	= iso_connected,
	.disconnected	= iso_disconnected,
	.sent		= iso_sent,
};

static struct bt_iso_chan_io_qos iso_tx = {
	.sdu = CONFIG_BT_ISO_TX_MTU,
	.phy = BT_GAP_LE_PHY_2M,
	.rtn = 1,
};

static struct bt_iso_chan_qos iso_qos = {
	.tx = &iso_tx,
	.rx = NULL,
};

struct iso_create_item{
	struct k_work iso_create_work;
	int index;
	struct bt_conn *conn;
};

void iso_create_work_handler(struct k_work *work)
{
	int iso_err;
	struct iso_create_item *item = CONTAINER_OF(work, struct iso_create_item, iso_create_work);
	printk("Creating ISO channel for conn %d\n", item->index);

	struct bt_iso_connect_param connect_param;
	connect_param.acl = item->conn;
	connect_param.iso_chan = &iso_chan[item->index];

	iso_err = bt_iso_chan_connect(&connect_param, 1);

	if (iso_err) {
		printk("Failed to connect iso (%d)\n", iso_err);
		k_sleep(K_MSEC(100));
		k_work_submit(&item->iso_create_work);
	}
}



static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];



	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s %u %s\n", addr, err, bt_hci_err_to_str(err));

		bt_conn_unref(peripheral_conn[conn_num_get(conn)]);
		peripheral_conn[conn_num_get(conn)] = NULL;

		start_scan();
		return;
	}


	printk("Connected: %s\n", addr);
	static struct iso_create_item item;
	item.index = conn_num_get(conn);
	item.conn = conn;
	k_work_init(&item.iso_create_work, iso_create_work_handler);
	k_work_submit(&item.iso_create_work);

	if (all_conn_connected()) {
		printk("All connections established\n");
	} else {
		start_scan();
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

	bt_conn_unref(peripheral_conn[conn_num_get(conn)]);
	peripheral_conn[conn_num_get(conn)] = NULL;

	start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

int main(void)
{
	int err;
	struct bt_iso_chan *channels[2];
	struct bt_iso_cig_param param;
	struct bt_iso_cig *cig;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}


	printk("Bluetooth initialized\n");

	iso_chan[0].ops = &iso_ops;
	iso_chan[0].qos = &iso_qos;
	iso_chan[1].ops = &iso_ops;
	iso_chan[1].qos = &iso_qos;
#if defined(CONFIG_BT_SMP)
	iso_chan[0].required_sec_level = BT_SECURITY_L2;
	iso_chan[1].required_sec_level = BT_SECURITY_L2;
#endif /* CONFIG_BT_SMP */

	channels[0] = &iso_chan[0];
	channels[1] = &iso_chan[1];
	param.cis_channels = channels;
	param.num_cis = ARRAY_SIZE(channels);
	param.sca = BT_GAP_SCA_UNKNOWN;
	param.packing = 0;
	param.framing = 0;
	param.c_to_p_latency = latency_ms; /* ms */
	param.p_to_c_latency = latency_ms; /* ms */
	param.c_to_p_interval = interval_us; /* us */
	param.p_to_c_interval = interval_us; /* us */

	err = bt_iso_cig_create(&param, &cig);

	if (err != 0) {
		printk("Failed to create CIG (%d)\n", err);
		return 0;
	}

	start_scan();

	k_work_init_delayable(&iso_send_work, iso_timer_timeout);
	return 0;
}
