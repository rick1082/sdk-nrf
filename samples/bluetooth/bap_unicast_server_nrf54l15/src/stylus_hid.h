/*
 * Copyright (c) 2021-2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef HID_H
#define HID_H

#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/hids.h>

#define BASE_USB_HID_SPEC_VERSION 0x0101

/* Number of pixels by which the cursor is moved when a button is pushed. */
#define MOVEMENT_SPEED		   10
/* Number of input reports in this application. */
#define INPUT_REPORT_COUNT	   3
/* Length of Mouse Input Report containing button data. */
#define INPUT_REP_BUTTONS_LEN	   3
/* Length of Mouse Input Report containing movement data. */
#define INPUT_REP_MOVEMENT_LEN	   3
/* Length of Mouse Input Report containing media player data. */
#define INPUT_REP_MEDIA_PLAYER_LEN 1
/* Index of Mouse Input Report containing button data. */
#define INPUT_REP_BUTTONS_INDEX	   0
/* Index of Mouse Input Report containing movement data. */
#define INPUT_REP_MOVEMENT_INDEX   1
/* Index of Mouse Input Report containing media player data. */
#define INPUT_REP_MPLAYER_INDEX	   2
/* Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_BUTTONS_ID   1
/* Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MOVEMENT_ID  2
/* Id of reference to Mouse Input Report containing media player data. */
#define INPUT_REP_REF_MPLAYER_ID   3

/** @brief Insert a connection object into the HID service.
 * 
 * This function is called when a new connection is established.
 *
 * @param hids_obj Pointer to the HID service object.
 * @param conn Pointer to the Bluetooth connection object.
 */
void stylus_hid_insert_conn_object(struct bt_hids *hids_obj,struct bt_conn *conn);

/** @brief Remove a connection object from the HID service.
 *
 * This function is called when a connection is terminated.
 *
 * @param hids_obj Pointer to the HID service object.
 * @param conn Pointer to the Bluetooth connection object.
 */
void stylus_hid_remove_conn_object(struct bt_hids *hids_obj, struct bt_conn *conn);

/** @brief Send mouse movement data to the HID service.
 *
 * This function is called to send mouse movement data to the HID service.
 *
 * @param hids_obj Pointer to the HID service object.
 * @param x_delta Change in the X axis.
 * @param y_delta Change in the Y axis.
 */
void stylus_hid_mouse_movement_send(struct bt_hids *hids_obj, int16_t x_delta, int16_t y_delta);

/** @brief Initialize the HID service.
 *
 * This function is called to initialize the HID service.
 *
 * @param hids_obj Pointer to the HID service object.
 */
void stylus_hid_init(struct bt_hids *hids_obj);

#endif