/*
 * Copyright (c) 2021-2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "stylus_hid.h"
#include <zephyr/sys/byteorder.h>
#include <bluetooth/services/hids.h>

static struct conn_mode {
	struct bt_conn *conn;
	bool in_boot_mode;
} conn_mode[CONFIG_BT_HIDS_MAX_CLIENT_COUNT];

void stylus_hid_remove_conn_object(struct bt_hids *hids_obj, struct bt_conn *conn)
{
    int err;

	err = bt_hids_disconnected(hids_obj, conn);
	if (err) {
		printk("Failed to notify HID service about disconnection\n");
	}    

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (conn_mode[i].conn == conn) {
			conn_mode[i].conn = NULL;
			conn_mode[i].in_boot_mode = false;
			return;
		}
	}
}

void stylus_hid_insert_conn_object(struct bt_hids *hids_obj, struct bt_conn *conn)
{
    int err;

	err = bt_hids_connected(hids_obj, conn);
	if (err) {
		printk("Failed to notify HID service about connection\n");
		return;
	}

	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (!conn_mode[i].conn) {
			conn_mode[i].conn = conn;
			conn_mode[i].in_boot_mode = false;

			return;
		}
	}

	printk("Connection object could not be inserted %p\n", conn);
}

static void hids_pm_evt_handler(enum bt_hids_pm_evt evt, struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	size_t i;

	for (i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {
		if (conn_mode[i].conn == conn) {
			break;
		}
	}

	if (i >= CONFIG_BT_HIDS_MAX_CLIENT_COUNT) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	switch (evt) {
	case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
		printk("Boot mode entered %s\n", addr);
		conn_mode[i].in_boot_mode = true;
		break;

	case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
		printk("Report mode entered %s\n", addr);
		conn_mode[i].in_boot_mode = false;
		break;

	default:
		break;
	}
}

void stylus_hid_init(struct bt_hids *hids_obj)
{
	int err;
	struct bt_hids_init_param hids_init_param = {0};
	struct bt_hids_inp_rep *hids_inp_rep;
	static const uint8_t mouse_movement_mask[DIV_ROUND_UP(INPUT_REP_MOVEMENT_LEN, 8)] = {0};

	static const uint8_t report_map[] = {
		0x05, 0x01, /* Usage Page (Generic Desktop) */
		0x09, 0x02, /* Usage (Mouse) */

		0xA1, 0x01, /* Collection (Application) */

		/* Report ID 1: Mouse buttons + scroll/pan */
		0x85, 0x01,	  /* Report Id 1 */
		0x09, 0x01,	  /* Usage (Pointer) */
		0xA1, 0x00,	  /* Collection (Physical) */
		0x95, 0x05,	  /* Report Count (3) */
		0x75, 0x01,	  /* Report Size (1) */
		0x05, 0x09,	  /* Usage Page (Buttons) */
		0x19, 0x01,	  /* Usage Minimum (01) */
		0x29, 0x05,	  /* Usage Maximum (05) */
		0x15, 0x00,	  /* Logical Minimum (0) */
		0x25, 0x01,	  /* Logical Maximum (1) */
		0x81, 0x02,	  /* Input (Data, Variable, Absolute) */
		0x95, 0x01,	  /* Report Count (1) */
		0x75, 0x03,	  /* Report Size (3) */
		0x81, 0x01,	  /* Input (Constant) for padding */
		0x75, 0x08,	  /* Report Size (8) */
		0x95, 0x01,	  /* Report Count (1) */
		0x05, 0x01,	  /* Usage Page (Generic Desktop) */
		0x09, 0x38,	  /* Usage (Wheel) */
		0x15, 0x81,	  /* Logical Minimum (-127) */
		0x25, 0x7F,	  /* Logical Maximum (127) */
		0x81, 0x06,	  /* Input (Data, Variable, Relative) */
		0x05, 0x0C,	  /* Usage Page (Consumer) */
		0x0A, 0x38, 0x02, /* Usage (AC Pan) */
		0x95, 0x01,	  /* Report Count (1) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0xC0,		  /* End Collection (Physical) */

		/* Report ID 2: Mouse motion */
		0x85, 0x02,	  /* Report Id 2 */
		0x09, 0x01,	  /* Usage (Pointer) */
		0xA1, 0x00,	  /* Collection (Physical) */
		0x75, 0x0C,	  /* Report Size (12) */
		0x95, 0x02,	  /* Report Count (2) */
		0x05, 0x01,	  /* Usage Page (Generic Desktop) */
		0x09, 0x30,	  /* Usage (X) */
		0x09, 0x31,	  /* Usage (Y) */
		0x16, 0x01, 0xF8, /* Logical maximum (2047) */
		0x26, 0xFF, 0x07, /* Logical minimum (-2047) */
		0x81, 0x06,	  /* Input (Data, Variable, Relative) */
		0xC0,		  /* End Collection (Physical) */
		0xC0,		  /* End Collection (Application) */

		/* Report ID 3: Advanced buttons */
		0x05, 0x0C, /* Usage Page (Consumer) */
		0x09, 0x01, /* Usage (Consumer Control) */
		0xA1, 0x01, /* Collection (Application) */
		0x85, 0x03, /* Report Id (3) */
		0x15, 0x00, /* Logical minimum (0) */
		0x25, 0x01, /* Logical maximum (1) */
		0x75, 0x01, /* Report Size (1) */
		0x95, 0x01, /* Report Count (1) */

		0x09, 0xCD,	  /* Usage (Play/Pause) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0x0A, 0x83, 0x01, /* Usage (Consumer Control Configuration) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0x09, 0xB5,	  /* Usage (Scan Next Track) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0x09, 0xB6,	  /* Usage (Scan Previous Track) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */

		0x09, 0xEA,	  /* Usage (Volume Down) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0x09, 0xE9,	  /* Usage (Volume Up) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0x0A, 0x25, 0x02, /* Usage (AC Forward) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0x0A, 0x24, 0x02, /* Usage (AC Back) */
		0x81, 0x06,	  /* Input (Data,Value,Relative,Bit Field) */
		0xC0		  /* End Collection */
	};

	hids_init_param.rep_map.data = report_map;
	hids_init_param.rep_map.size = sizeof(report_map);

	hids_init_param.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
	hids_init_param.info.b_country_code = 0x00;
	hids_init_param.info.flags = (BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE);

	hids_inp_rep = &hids_init_param.inp_rep_group_init.reports[0];
	hids_inp_rep->size = INPUT_REP_BUTTONS_LEN;
	hids_inp_rep->id = INPUT_REP_REF_BUTTONS_ID;
	hids_init_param.inp_rep_group_init.cnt++;

	hids_inp_rep++;
	hids_inp_rep->size = INPUT_REP_MOVEMENT_LEN;
	hids_inp_rep->id = INPUT_REP_REF_MOVEMENT_ID;
	hids_inp_rep->rep_mask = mouse_movement_mask;
	hids_init_param.inp_rep_group_init.cnt++;

	hids_inp_rep++;
	hids_inp_rep->size = INPUT_REP_MEDIA_PLAYER_LEN;
	hids_inp_rep->id = INPUT_REP_REF_MPLAYER_ID;
	hids_init_param.inp_rep_group_init.cnt++;

	hids_init_param.is_mouse = true;
	hids_init_param.pm_evt_handler = hids_pm_evt_handler;

	err = bt_hids_init(hids_obj, &hids_init_param);
	__ASSERT(err == 0, "HIDS initialization failed\n");
}

void stylus_hid_mouse_movement_send(struct bt_hids *hids_obj, int16_t x_delta, int16_t y_delta)
{
	for (size_t i = 0; i < CONFIG_BT_HIDS_MAX_CLIENT_COUNT; i++) {

		if (!conn_mode[i].conn) {
			continue;
		}

		if (conn_mode[i].in_boot_mode) {
			x_delta = MAX(MIN(x_delta, SCHAR_MAX), SCHAR_MIN);
			y_delta = MAX(MIN(y_delta, SCHAR_MAX), SCHAR_MIN);

			bt_hids_boot_mouse_inp_rep_send(hids_obj, conn_mode[i].conn, NULL,
							(int8_t)x_delta, (int8_t)y_delta, NULL);
		} else {
			uint8_t x_buff[2];
			uint8_t y_buff[2];
			uint8_t buffer[INPUT_REP_MOVEMENT_LEN];

			int16_t x = MAX(MIN(x_delta, 0x07ff), -0x07ff);
			int16_t y = MAX(MIN(y_delta, 0x07ff), -0x07ff);

			/* Convert to little-endian. */
			sys_put_le16(x, x_buff);
			sys_put_le16(y, y_buff);

			/* Encode report. */
			BUILD_ASSERT(sizeof(buffer) == 3,
				     "Only 2 axis, 12-bit each, are supported");

			buffer[0] = x_buff[0];
			buffer[1] = (y_buff[0] << 4) | (x_buff[1] & 0x0f);
			buffer[2] = (y_buff[1] << 4) | (y_buff[0] >> 4);

			bt_hids_inp_rep_send(hids_obj, conn_mode[i].conn, INPUT_REP_MOVEMENT_INDEX,
					     buffer, sizeof(buffer), NULL);
		}
	}
}