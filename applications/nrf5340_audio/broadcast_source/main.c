/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "streamctrl.h"

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#include "nrf5340_audio_common.h"
#include "nrf5340_audio_dk.h"
#include "broadcast_source.h"
#include "led.h"
#include "button_assignments.h"
#include "macros_common.h"
#include "audio_system.h"
#include "bt_mgmt.h"
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/gatt_dm.h>
#include <stdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

ZBUS_SUBSCRIBER_DEFINE(button_evt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);

ZBUS_MSG_SUBSCRIBER_DEFINE(le_audio_evt_sub);

ZBUS_CHAN_DECLARE(button_chan);
ZBUS_CHAN_DECLARE(le_audio_chan);
ZBUS_CHAN_DECLARE(bt_mgmt_chan);
ZBUS_CHAN_DECLARE(sdu_ref_chan);

ZBUS_OBS_DECLARE(sdu_ref_msg_listen);

static struct k_thread button_msg_sub_thread_data;
static struct k_thread le_audio_msg_sub_thread_data;

static k_tid_t button_msg_sub_thread_id;
static k_tid_t le_audio_msg_sub_thread_id;

K_THREAD_STACK_DEFINE(button_msg_sub_thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE);
K_THREAD_STACK_DEFINE(le_audio_msg_sub_thread_stack, CONFIG_LE_AUDIO_MSG_SUB_STACK_SIZE);

static enum stream_state strm_state = STATE_PAUSED;

static struct bt_conn *nus_conn[CONFIG_BT_MAX_CONN];
static struct bt_nus_client *nus[CONFIG_BT_MAX_CONN];
static struct bt_nus_client nus_client[CONFIG_BT_MAX_CONN];
struct k_work_delayable dummy_data_send_work;
#define REMOTE_DEVICE_NAME_PEER "Nordic_UART_Service"
#define REMOTE_DEVICE_NAME_PEER_LEN (sizeof(REMOTE_DEVICE_NAME_PEER) - 1)

/* Function for handling all stream state changes */
static void stream_state_set(enum stream_state stream_state_new)
{
	strm_state = stream_state_new;
}

char dummy_string[] = "Hello from nRF5340\n\r";
static void work_dummy_data_send(struct k_work *work)
{
		int ret;
		for (int i = 0; i < CONFIG_BT_MAX_CONN; i++){
			if (nus_conn[i] != NULL){
				ret = bt_nus_client_send(&nus_client[i], dummy_string, sizeof(dummy_string));
				LOG_INF("Sending dummy string to server %d, ret = %d", i, ret);
			}	
		}
}

/**
 * @brief	Handle button activity.
 */

static void button_msg_sub_thread(void)
{
	int ret;
	const struct zbus_channel *chan;

	while (1) {
		ret = zbus_sub_wait(&button_evt_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		struct button_msg msg;

		ret = zbus_chan_read(chan, &msg, ZBUS_READ_TIMEOUT_MS);
		ERR_CHK(ret);

		LOG_DBG("Got btn evt from queue - id = %d, action = %d", msg.button_pin,
			msg.button_action);

		if (msg.button_action != BUTTON_PRESS) {
			LOG_WRN("Unhandled button action");
			return;
		}

		switch (msg.button_pin) {
		case BUTTON_PLAY_PAUSE:
			if (strm_state == STATE_STREAMING) {
				ret = broadcast_source_stop();
				if (ret) {
					LOG_WRN("Failed to stop broadcaster: %d", ret);
				}
			} else if (strm_state == STATE_PAUSED) {
				ret = broadcast_source_start(NULL);
				if (ret) {
					LOG_WRN("Failed to start broadcaster: %d", ret);
				}
			} else {
				LOG_WRN("In invalid state: %d", strm_state);
			}

			break;

		case BUTTON_5:
			k_work_reschedule(&dummy_data_send_work, K_MSEC(10));
			break;

		case BUTTON_4:
			if (IS_ENABLED(CONFIG_AUDIO_TEST_TONE)) {
				if (strm_state != STATE_STREAMING) {
					LOG_WRN("Not in streaming state");
					break;
				}

				ret = audio_system_encode_test_tone_step();
				if (ret) {
					LOG_WRN("Failed to play test tone, ret: %d", ret);
				}

				break;
			}

			break;

		default:
			LOG_WRN("Unexpected/unhandled button id: %d", msg.button_pin);
		}

		STACK_USAGE_PRINT("button_msg_thread", &button_msg_sub_thread_data);
	}
}

/**
 * @brief	Handle Bluetooth LE audio events.
 */
static void le_audio_msg_sub_thread(void)
{
	int ret;
	const struct zbus_channel *chan;

	while (1) {
		struct le_audio_msg msg;

		ret = zbus_sub_wait_msg(&le_audio_evt_sub, &chan, &msg, K_FOREVER);
		ERR_CHK(ret);

		LOG_DBG("Received event = %d, current state = %d", msg.event, strm_state);

		switch (msg.event) {
		case LE_AUDIO_EVT_STREAMING:
			LOG_DBG("LE audio evt streaming");

			audio_system_encoder_start();

			if (strm_state == STATE_STREAMING) {
				LOG_DBG("Got streaming event in streaming state");
				break;
			}

			audio_system_start();
			stream_state_set(STATE_STREAMING);
			ret = led_blink(LED_APP_1_BLUE);
			ERR_CHK(ret);

			break;

		case LE_AUDIO_EVT_NOT_STREAMING:
			LOG_DBG("LE audio evt not_streaming");

			audio_system_encoder_stop();

			if (strm_state == STATE_PAUSED) {
				LOG_DBG("Got not_streaming event in paused state");
				break;
			}

			stream_state_set(STATE_PAUSED);
			audio_system_stop();
			ret = led_on(LED_APP_1_BLUE);
			ERR_CHK(ret);

			break;

		default:
			LOG_WRN("Unexpected/unhandled le_audio event: %d", msg.event);

			break;
		}

		STACK_USAGE_PRINT("le_audio_msg_thread", &le_audio_msg_sub_thread_data);
	}
}

/**
 * @brief	Create zbus subscriber threads.
 *
 * @return	0 for success, error otherwise.
 */
static int zbus_subscribers_create(void)
{
	int ret;

	button_msg_sub_thread_id = k_thread_create(
		&button_msg_sub_thread_data, button_msg_sub_thread_stack,
		CONFIG_BUTTON_MSG_SUB_STACK_SIZE, (k_thread_entry_t)button_msg_sub_thread, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_BUTTON_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(button_msg_sub_thread_id, "BUTTON_MSG_SUB");
	if (ret) {
		LOG_ERR("Failed to create button_msg thread");
		return ret;
	}

	le_audio_msg_sub_thread_id = k_thread_create(
		&le_audio_msg_sub_thread_data, le_audio_msg_sub_thread_stack,
		CONFIG_LE_AUDIO_MSG_SUB_STACK_SIZE, (k_thread_entry_t)le_audio_msg_sub_thread, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_LE_AUDIO_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(le_audio_msg_sub_thread_id, "LE_AUDIO_MSG_SUB");
	if (ret) {
		LOG_ERR("Failed to create le_audio_msg thread");
		return ret;
	}

	ret = zbus_chan_add_obs(&sdu_ref_chan, &sdu_ref_msg_listen, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add timestamp listener");
		return ret;
	}

	return 0;
}

static uint8_t ble_data_received(struct bt_nus_client *nus, const uint8_t *data, uint16_t len)
{
	LOG_HEXDUMP_INF(data, len, "NUS received:");
	return BT_GATT_ITER_CONTINUE;
}

struct bt_nus_client_init_param init = { .cb = {
							.received = ble_data_received,
						} };

static int channel_index_get(const struct bt_conn *conn, uint8_t *index)
{
	if (conn == NULL) {
		LOG_ERR("No connection provided");
		return -EINVAL;
	}

	for (int i = 0; i < ARRAY_SIZE(nus_conn); i++) {
		if (nus_conn[i] == conn) {
			*index = i;
			return 0;
		}
	}

	LOG_WRN("Connection not found");

	return -EINVAL;
}

static void discovery_complete(struct bt_gatt_dm *dm, void *context)
{
	int ret;
	uint8_t channel_index;

	ret = channel_index_get(bt_gatt_dm_conn_get(dm), &channel_index);
	if (ret) {
		LOG_ERR("Channel index not found");
	}

	LOG_INF("Service discovery completed for nus[%d]", channel_index);
	nus[channel_index] = context;

	bt_gatt_dm_data_print(dm);

	bt_nus_handles_assign(dm, nus[channel_index]);
	bt_nus_subscribe_receive(nus[channel_index]);

	bt_gatt_dm_data_release(dm);
}

static void discovery_service_not_found(struct bt_conn *conn, void *context)
{
	LOG_INF("Service not found");
}

static void discovery_error(struct bt_conn *conn, int err, void *context)
{
	LOG_WRN("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found = discovery_error,
};

static void gatt_discover(struct bt_conn *conn)
{
	int ret;
	uint8_t channel_index;

	ret = channel_index_get(conn, &channel_index);
	if (ret) {
		LOG_ERR("Channel index not found");
	}

	LOG_INF("Discovering GATT database for %p, index = %d", (void *) conn, channel_index);
	ret = bt_gatt_dm_start(conn, BT_UUID_NUS_SERVICE, &discovery_cb,
			       &nus_client[channel_index]);
	if (ret) {
		LOG_ERR("could not start the discovery procedure, error "
			"code: %d", ret);
	}
}

/**
 * @brief	Zbus listener to receive events from bt_mgmt.
 *
 * @param[in]	chan	Zbus channel.
 *
 * @note	Will in most cases be called from BT_RX context,
 *		so there should not be too much processing done here.
 */
static void bt_mgmt_evt_handler(const struct zbus_channel *chan)
{
	int ret;
	const struct bt_mgmt_msg *msg;

	msg = zbus_chan_const_msg(chan);

	switch (msg->event) {
	case BT_MGMT_EXT_ADV_WITH_PA_READY:
		LOG_INF("Ext adv ready");

		ret = broadcast_source_start(msg->ext_adv);
		if (ret) {
			LOG_ERR("Failed to start broadcaster: %d", ret);
		}

		break;

	case BT_MGMT_CONNECTED:
		LOG_INF("BT_MGMT_CONNECTED");
		int i;

		for (i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			if (nus_conn[i] == NULL) {
				nus_conn[i] = msg->conn;
				break;
			}
		}
		gatt_discover(msg->conn);
		break;

	case BT_MGMT_SECURITY_CHANGED:
		LOG_INF("BT_MGMT_SECURITY_CHANGED");
		break;

	case BT_MGMT_DISCONNECTED:
		for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
			if (nus_conn[i] == msg->conn) {
				nus_conn[i] = NULL;
				break;
			}
		}
		LOG_INF("BT_MGMT_DISCONNECTED");
		break;

	default:
		LOG_WRN("Unexpected/unhandled bt_mgmt event: %d", msg->event);
		break;
	}
}

ZBUS_LISTENER_DEFINE(bt_mgmt_evt_listen, bt_mgmt_evt_handler);

/**
 * @brief	Link zbus producers and observers.
 *
 * @return	0 for success, error otherwise.
 */
static int zbus_link_producers_observers(void)
{
	int ret;

	if (!IS_ENABLED(CONFIG_ZBUS)) {
		return -ENOTSUP;
	}

	ret = zbus_chan_add_obs(&button_chan, &button_evt_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add button sub");
		return ret;
	}

	ret = zbus_chan_add_obs(&le_audio_chan, &le_audio_evt_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add le_audio sub");
		return ret;
	}

	ret = zbus_chan_add_obs(&bt_mgmt_chan, &bt_mgmt_evt_listen, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add bt_mgmt listener");
		return ret;
	}

	return 0;
}

uint8_t stream_state_get(void)
{
	return strm_state;
}

void streamctrl_send(void const *const data, size_t size, uint8_t num_ch)
{
	int ret;
	static int prev_ret;

	struct le_audio_encoded_audio enc_audio = {.data = data, .size = size, .num_ch = num_ch};

	if (strm_state == STATE_STREAMING) {
		ret = broadcast_source_send(enc_audio);

		if (ret != 0 && ret != prev_ret) {
			if (ret == -ECANCELED) {
				LOG_WRN("Sending operation cancelled");
			} else {
				LOG_WRN("Problem with sending LE audio data, ret: %d", ret);
			}
		}

		prev_ret = ret;
	}
}


static int nus_client_init(void)
{
	int err;
	struct bt_nus_client_init_param init = { .cb = {
							 .received = ble_data_received,
						 } };

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		err = bt_nus_client_init(&nus_client[i], &init);
		if (err) {
			LOG_ERR("NUS Client initialization failed (err %d)", err);
			return err;
		}
	}

	LOG_INF("NUS Client module initialized");
	return err;
}

int main(void)
{
	int ret;
	static const struct bt_data *ext_adv;
	static const struct bt_data *per_adv;

	LOG_DBG("nRF5340 APP core started");

	k_work_init_delayable(&dummy_data_send_work, work_dummy_data_send);

	ret = nrf5340_audio_dk_init();
	ERR_CHK(ret);

	ret = nrf5340_audio_common_init();
	ERR_CHK(ret);

	size_t ext_adv_size = 0;
	size_t per_adv_size = 0;

	ret = zbus_subscribers_create();
	ERR_CHK_MSG(ret, "Failed to create zbus subscriber threads");

	ret = zbus_link_producers_observers();
	ERR_CHK_MSG(ret, "Failed to link zbus producers and observers");

	ret = broadcast_source_enable();
	ERR_CHK_MSG(ret, "Failed to enable broadcaster");

	ret = audio_system_config_set(
		bt_audio_codec_cfg_freq_to_freq_hz(CONFIG_BT_AUDIO_PREF_SAMPLE_RATE_VALUE),
		CONFIG_BT_AUDIO_BITRATE_BROADCAST_SRC, VALUE_NOT_SET);
	ERR_CHK_MSG(ret, "Failed to set sample- and bitrate");

	broadcast_source_adv_get(&ext_adv, &ext_adv_size, &per_adv, &per_adv_size);

	ret = nus_client_init();
	ERR_CHK_MSG(ret, "Failed to initialize NUS client");

	ret = bt_mgmt_adv_start(ext_adv, ext_adv_size, per_adv, per_adv_size, false);
	ERR_CHK_MSG(ret, "Failed to start advertiser");

	ret = bt_mgmt_scan_start(0, 0, BT_MGMT_SCAN_TYPE_CONN, REMOTE_DEVICE_NAME_PEER,
				 BRDCAST_ID_NOT_USED);
	ERR_CHK_MSG(ret, "Failed to start scanning");
	


	return 0;
}
