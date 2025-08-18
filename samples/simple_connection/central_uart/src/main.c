/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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
#include <zephyr/sys/byteorder.h>

#include <bluetooth/services/nus.h>

static struct bt_uuid_128 discover_uuid = BT_UUID_INIT_128(0);
static struct bt_uuid_16 discover_cccd_uuid = BT_UUID_INIT_16(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

static void start_scan(void);

static struct bt_conn *default_conn;
static uint16_t nus_write_handle = 0xffff;

static bool device_name_check(struct bt_data *data, void *user_data)
{
	int ret;
	bt_addr_le_t *addr = user_data;
	char addr_string[BT_ADDR_LE_STR_LEN];
	char srch_name[] = CONFIG_BT_DEVICE_NAME;
	/* We only care about LTVs with name */
	if (data->type == BT_DATA_NAME_COMPLETE || data->type == BT_DATA_NAME_SHORTENED) {
		size_t srch_name_size = strlen(srch_name);
		//printk("%s %d\n", data->data, data->data_len);
		if ((data->data_len == srch_name_size) &&
		    (memcmp(srch_name, data->data, srch_name_size) == 0)) {

			printk("Device found: %s\n", srch_name);

			ret = bt_le_scan_stop();
			if (ret) {
				printk("Stop scan failed: %d\n", ret);
			}

			bt_addr_le_to_str(addr, addr_string, BT_ADDR_LE_STR_LEN);

			printk("Creating connection to device: %s\n", addr_string);

			ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT,
						&default_conn);
			if (ret) {
				printk("Could not init connection: %d\n", ret);
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
	if (default_conn) {
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
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

static uint8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	printk("[NOTIFICATION] data %p length %u\n", data, length);
	printk("%s\n", (char *)data);

	return BT_GATT_ITER_CONTINUE;
}

static int nus_write(const char *data, uint16_t length)
{
	int err;

	if (!default_conn || nus_write_handle == 0xffff) {
		printk("Not connected or NUS write handle not set\n");
		return -EIO;
	}

	err = bt_gatt_write_without_response(default_conn, nus_write_handle, data, length, false);
	if (err) {
		printk("Write failed (err %d)\n", err);
		return err;
	}
	return 0;
}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_NUS_SERVICE)) {
		printk("NUS service found\n");
		memcpy(&discover_uuid, BT_UUID_NUS_TX, sizeof(discover_uuid));
		discover_params.uuid = &discover_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
		
	} else if (!bt_uuid_cmp(discover_params.uuid,
				BT_UUID_NUS_TX)) {
		printk("NUS TX service found\n");
		memcpy(&discover_cccd_uuid, BT_UUID_GATT_CCC, sizeof(discover_cccd_uuid));
		discover_params.uuid = &discover_cccd_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
				BT_UUID_GATT_CCC)) {
		subscribe_params.notify = notify_func;
		subscribe_params.value = BT_GATT_CCC_NOTIFY;
		subscribe_params.ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_params);
		if (err && err != -EALREADY) {
			printk("Subscribe failed (err %d)\n", err);
		} else {
			printk("[SUBSCRIBED]\n");
		}

		memcpy(&discover_uuid, BT_UUID_NUS_RX, sizeof(discover_uuid));
		discover_params.uuid = &discover_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
				BT_UUID_NUS_RX)) {
		printk("NUS RX service found\n");
		nus_write_handle = bt_gatt_attr_value_handle(attr);
		return BT_GATT_ITER_STOP;
	} else {
		printk("Unknown attribute found\n");
		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_STOP;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s %u %s\n", addr, err, bt_hci_err_to_str(err));

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	if (conn != default_conn) {
		return;
	}

	printk("Connected: %s\n", addr);

	memcpy(&discover_uuid, BT_UUID_NUS_SERVICE, sizeof(discover_uuid));
	discover_params.uuid = &discover_uuid.uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	err = bt_gatt_discover(default_conn, &discover_params);
	if (err) {
		printk("Discover failed(err %d)\n", err);
		return;
	}
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

	start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");
	const char *hello_world = "Hello World!";
	start_scan();
	while(1){
		nus_write(hello_world, strlen(hello_world));
		k_sleep(K_MSEC(1000));
	}
	return 0;
}
