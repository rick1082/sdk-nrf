/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BT_CONTENT_CTRL_H_
#define _BT_CONTENT_CTRL_H_

#include <zephyr/bluetooth/conn.h>

/**
 * @brief	Send the start request for content transmission.
 *
 * @param[in]	conn	Pointer to the connection to control; can be NULL.
 *
 * @return	0 for success, error otherwise.
 */
int bt_content_ctrl_start(struct bt_conn *conn);

/**
 * @brief	Send the stop request for content transmission.
 *
 * @param[in]	conn	Pointer to the connection to control; can be NULL.
 *
 * @return	0 for success, error otherwise.
 */
int bt_content_ctrl_stop(struct bt_conn *conn);

/**
 * @brief	Handle disconnected connection for the content control services.
 *
 * @param[in]	conn	Pointer to the disconnected connection.
 *
 * @return	0 for success, error otherwise.
 */
int bt_content_ctrl_conn_disconnected(struct bt_conn *conn);

/**
 * @brief	Discover the content control services for the given connection pointer.
 *
 * @param[in]	conn	Pointer to the connection on which to discover the services.
 *
 * @return	0 for success, error otherwise.
 */
int bt_content_ctrl_discover(struct bt_conn *conn);

/**
 * @brief	Get advertising data from this module.
 *
 * @note	This partial data is used to build a complete extended advertising packet.
 *
 * @param [in/out] uuid_buf Buffer being populated with UUIDs.
 * @param [in/out] adv_buf Buffer being populated with ext adv elements.
 * @param [in] adv_buf_vacant Number of vacant elements in @p adv_buf.
 *
 * @return	Negative values for errors or num elements added to @p adv_buf.
 */
int bt_content_ctrl_adv_get(struct net_buf_simple *uuid_buf, struct bt_data *adv_buf,
			    uint8_t adv_buf_vacant);

/**
 * @brief	Initialize the content control module.
 *
 * @return	0 for success, error otherwise.
 */
int bt_content_ctrl_init(void);

#endif /* _BT_CONTENT_CTRL_H_ */
