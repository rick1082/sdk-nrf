/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _UNICAST_CLIENT_H_
#define _UNICAST_CLIENT_H_

#include "le_audio.h"

#include <zephyr/bluetooth/audio/audio.h>
#include <audio_defines.h>

enum unicast_discover_dir {
	UNICAST_SERVER_SINK = BIT(BT_AUDIO_DIR_SINK),
	UNICAST_SERVER_SOURCE = BIT(BT_AUDIO_DIR_SOURCE),
	UNICAST_SERVER_BIDIR = (BIT(BT_AUDIO_DIR_SINK) | BIT(BT_AUDIO_DIR_SOURCE))
};

/**
 * @brief Start service discover for the Bluetooth LE Audio unicast (CIS) server.
 *
 * @param[in] conn Pointer to the connection object.
 * @param[in] dir Direction of the stream.
 *
 * @return 0 for success, error otherwise.
 */
int unicast_client_discover(struct bt_conn *conn, enum unicast_discover_dir dir);

/**
 * @brief	Handle a disconnection cleanup from the Bluetooth LE Audio unicast (CIS) client.
 *
 * @param[in]	conn	Pointer to the connection object.
 */
void unicast_client_conn_disconnected(struct bt_conn *conn);

/**
 * @brief	Start the Bluetooth LE Audio unicast (CIS) client.
 *
 * @return	0 for success, error otherwise.
 */
int unicast_client_start(void);

/**
 * @brief	Stop the Bluetooth LE Audio unicast (CIS) client.
 *
 * @return	0 for success, error otherwise.
 */
int unicast_client_stop(void);

/**
 * @brief	Send data from the LE Audio unicast (CIS) client, if configured as a sink.
 *
 * @param[in]	enc_audio	Encoded audio struct.
 *
 * @return	0 for success, error otherwise.
 */
int unicast_client_send(struct encoded_audio enc_audio);

/**
 * @brief	Enable the Bluetooth LE Audio unicast (CIS) client.
 *
 * @return	0 for success, error otherwise.
 */
int unicast_client_enable(le_audio_receive_cb recv_cb);

#endif /* _UNICAST_CLIENT_H_ */
