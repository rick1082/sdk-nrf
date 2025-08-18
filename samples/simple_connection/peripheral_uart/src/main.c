/*
 * Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>

#define DEVICE_NAME		CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN		(sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

static void notif_enabled(bool enabled, void *ctx)
{
	ARG_UNUSED(ctx);

	printk("%s() - %s\n", __func__, (enabled ? "Enabled" : "Disabled"));
}

static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(ctx);

	printk("%s() - Len: %d, Message: %.*s\n", __func__, len, len, (char *)data);
}

struct bt_nus_cb nus_listener = {
	.notif_enabled = notif_enabled,
	.received = received,
};

static struct bt_conn *current_conn;

static struct k_work adv_work;

static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		printk("Connection failed, err 0x%02x\n", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Connected %s\n", addr);

	current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s, reason 0x%02x\n", addr, reason);


	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	advertising_start();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
};

int main(void)
{
	int err;

	printk("Sample - Bluetooth Peripheral NUS\n");

	err = bt_nus_cb_register(&nus_listener, NULL);
	if (err) {
		printk("Failed to register NUS callback: %d\n", err);
		return err;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Failed to enable bluetooth: %d\n", err);
		return err;
	}

	k_work_init(&adv_work, adv_work_handler);
	advertising_start();
	printk("Initialization complete\n");

	while (true) {
		char hello_world[100];
		for(int i = 0; i < 100; i++) {
			hello_world[i] = 'A' + (i % 26);
		}
		hello_world[99] = '\0';

		k_sleep(K_SECONDS(3));

		err = bt_nus_send(NULL, hello_world, sizeof(hello_world));
		printk("Data send - Result: %d\n", err);

		if (err < 0 && (err != -EAGAIN) && (err != -ENOTCONN)) {
			return err;
		}
	}

	return 0;
}
