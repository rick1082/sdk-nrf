/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "streamctrl.h"

#include <zephyr/zbus/zbus.h>

#include "unicast_client.h"
#include "zbus_common.h"
#include "nrf5340_audio_dk.h"
#include "led.h"
#include "button_assignments.h"
#include "macros_common.h"
#include "audio_system.h"
#include "button_handler.h"
#include "bt_le_audio_tx.h"
#include "bt_mgmt.h"
#include "bt_rendering_and_capture.h"
#include "bt_content_ctrl.h"
#include "le_audio_rx.h"
#include "fw_info_app.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

static enum stream_state strm_state = STATE_PAUSED;

ZBUS_SUBSCRIBER_DEFINE(button_evt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);
ZBUS_SUBSCRIBER_DEFINE(content_control_evt_sub, CONFIG_CONTENT_CONTROL_MSG_SUB_QUEUE_SIZE);

ZBUS_MSG_SUBSCRIBER_DEFINE(le_audio_evt_sub);

ZBUS_CHAN_DECLARE(button_chan);
ZBUS_CHAN_DECLARE(le_audio_chan);
ZBUS_CHAN_DECLARE(bt_mgmt_chan);
ZBUS_CHAN_DECLARE(cont_media_chan);

static struct k_thread button_msg_sub_thread_data;
static struct k_thread le_audio_msg_sub_thread_data;
static struct k_thread content_control_msg_sub_thread_data;
static struct k_thread ble_qos_thread_data;

static k_tid_t button_msg_sub_thread_id;
static k_tid_t le_audio_msg_sub_thread_id;
static k_tid_t content_control_thread_id;
static k_tid_t ble_qos_thread_id;

#define BLE_QOS_THREAD_STACK_SIZE 2048

K_THREAD_STACK_DEFINE(button_msg_sub_thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE);
K_THREAD_STACK_DEFINE(le_audio_msg_sub_thread_stack, CONFIG_LE_AUDIO_MSG_SUB_STACK_SIZE);
K_THREAD_STACK_DEFINE(content_control_msg_sub_thread_stack,
		      CONFIG_CONTENT_CONTROL_MSG_SUB_STACK_SIZE);
K_THREAD_STACK_DEFINE(ble_qos_thread_stack, BLE_QOS_THREAD_STACK_SIZE);

/* Function for handling all stream state changes */
static void stream_state_set(enum stream_state stream_state_new)
{
	strm_state = stream_state_new;
}

static void content_control_msg_sub_thread(void)
{
	int ret;
	const struct zbus_channel *chan;

	while (1) {
		ret = zbus_sub_wait(&content_control_evt_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		struct content_control_msg msg;

		ret = zbus_chan_read(chan, &msg, ZBUS_READ_TIMEOUT_MS);
		ERR_CHK(ret);

		switch (msg.event) {
		case MEDIA_START:
			unicast_client_start(0);
			break;

		case MEDIA_STOP:
			unicast_client_stop(0);
			break;

		default:
			LOG_WRN("Unhandled event from content ctrl: %d", msg.event);
			break;
		}

		STACK_USAGE_PRINT("content_ctrl_msg_thread", &content_control_msg_sub_thread);
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
			if (IS_ENABLED(CONFIG_WALKIE_TALKIE_DEMO)) {
				LOG_WRN("Play/pause not supported in walkie-talkie mode");
				break;
			}

			if (strm_state == STATE_STREAMING) {
				ret = bt_content_ctrl_stop(NULL);
				if (ret) {
					LOG_WRN("Could not stop: %d", ret);
				}

			} else if (strm_state == STATE_PAUSED) {
				ret = bt_content_ctrl_start(NULL);
				if (ret) {
					LOG_WRN("Could not start: %d", ret);
				}

			} else {
				LOG_WRN("In invalid state: %d", strm_state);
			}

			break;

		case BUTTON_VOLUME_UP:
			ret = bt_r_and_c_volume_up();
			if (ret) {
				LOG_WRN("Failed to increase volume: %d", ret);
			}

			break;

		case BUTTON_VOLUME_DOWN:
			ret = bt_r_and_c_volume_down();
			if (ret) {
				LOG_WRN("Failed to decrease volume: %d", ret);
			}

			break;

		case BUTTON_4:
			if (IS_ENABLED(CONFIG_AUDIO_TEST_TONE)) {
				if (IS_ENABLED(CONFIG_WALKIE_TALKIE_DEMO)) {
					LOG_DBG("Test tone is disabled in walkie-talkie mode");
					break;
				}

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

		case BUTTON_5:
			if (IS_ENABLED(CONFIG_AUDIO_MUTE)) {
				ret = bt_r_and_c_volume_mute(false);
				if (ret) {
					LOG_WRN("Failed to mute, ret: %d", ret);
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
	uint32_t bitrate_bps;
	uint32_t sampling_rate_hz;
	const struct zbus_channel *chan;

	while (1) {
		struct le_audio_msg msg;

		ret = zbus_sub_wait_msg(&le_audio_evt_sub, &chan, &msg, K_FOREVER);
		ERR_CHK(ret);

		LOG_DBG("Received event = %d, current state = %d", msg.event, strm_state);

		switch (msg.event) {
		case LE_AUDIO_EVT_STREAMING:
			LOG_DBG("LE audio evt streaming");

			if (strm_state == STATE_STREAMING) {
				LOG_DBG("Got streaming event in streaming state");
				break;
			}

			if (msg.dir == BT_AUDIO_DIR_SINK) {
				audio_system_encoder_start();
			}

			audio_system_start();
			stream_state_set(STATE_STREAMING);

			ret = led_blink(LED_APP_1_BLUE);
			ERR_CHK(ret);
			break;

		case LE_AUDIO_EVT_NOT_STREAMING:
			LOG_DBG("LE audio evt not_streaming");

			if (strm_state == STATE_PAUSED) {
				LOG_DBG("Got not_streaming event in paused state");
				break;
			}

			if (msg.dir == BT_AUDIO_DIR_SINK) {
				audio_system_encoder_stop();
			}

			stream_state_set(STATE_PAUSED);
			audio_system_stop();

			ret = led_on(LED_APP_1_BLUE);
			ERR_CHK(ret);
			break;

		case LE_AUDIO_EVT_NO_VALID_CFG:
			LOG_WRN("No valid configurations found or CIS establishment failed, will "
				"disconnect");

			ret = bt_mgmt_conn_disconnect(msg.conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			if (ret) {
				LOG_ERR("Failed to disconnect: %d", ret);
			}

			break;

		case LE_AUDIO_EVT_CONFIG_RECEIVED:
			struct bt_conn_info conn_info;
			uint16_t interval = 0;

			ret = bt_conn_get_info(msg.conn, &conn_info);
			if (ret) {
				LOG_ERR("Failed to get conn info");
			} else {
				interval = conn_info.le.interval;
			}

			LOG_DBG("LE audio config received");

			ret = unicast_client_config_get(msg.conn, msg.dir, &bitrate_bps,
							&sampling_rate_hz);
			if (ret) {
				LOG_WRN("Failed to get config: %d", ret);
				break;
			}

			LOG_DBG("\tSampling rate: %d Hz", sampling_rate_hz);
			LOG_DBG("\tBitrate (compressed): %d bps", bitrate_bps);

			if (msg.dir == BT_AUDIO_DIR_SINK) {
				ret = audio_system_config_set(sampling_rate_hz, bitrate_bps,
							      VALUE_NOT_SET);
				ERR_CHK(ret);
			} else if (msg.dir == BT_AUDIO_DIR_SOURCE) {
				ret = audio_system_config_set(VALUE_NOT_SET, VALUE_NOT_SET,
							      sampling_rate_hz);
				ERR_CHK(ret);
			}

			break;

		case LE_AUDIO_EVT_COORD_SET_DISCOVERED:
			uint8_t num_conn = 0;
			uint8_t num_filled = 0;

			bt_mgmt_num_conn_get(&num_conn);

			if (msg.set_size > 0) {
				/* Check how many active connections we have for the given set */
				LOG_DBG("Setting SIRK");
				bt_mgmt_scan_sirk_set(msg.sirk);
				bt_mgmt_set_size_filled_get(&num_filled);

				LOG_INF("Set members found: %d of %d", num_filled, msg.set_size);

				if (num_filled == msg.set_size) {
					/* All devices in set found, clear SIRK before scanning */
					bt_mgmt_scan_sirk_set(NULL);
				}
			}

			if (num_conn < CONFIG_BT_MAX_CONN) {
				/* Room for more connections, start scanning again */
				ret = bt_mgmt_scan_start(0, 0, BT_MGMT_SCAN_TYPE_CONN, NULL,
							 BRDCAST_ID_NOT_USED);
				if (ret) {
					LOG_ERR("Failed to resume scanning: %d", ret);
				}
			}

			break;

		case LE_AUDIO_EVT_STREAM_SENT:
			/* Nothing to do. */
			break;

		default:
			LOG_WRN("Unexpected/unhandled le_audio event: %d", msg.event);
			break;
		}

		STACK_USAGE_PRINT("le_audio_msg_thread", &le_audio_msg_sub_thread_data);
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
	uint8_t num_conn = 0;

	msg = zbus_chan_const_msg(chan);
	bt_mgmt_num_conn_get(&num_conn);

	switch (msg->event) {
	case BT_MGMT_CONNECTED:
		/* NOTE: The string below is used by the Nordic CI system */
		LOG_INF("Connection event. Num connections: %u", num_conn);

		break;

	case BT_MGMT_SECURITY_CHANGED:
		LOG_INF("Security changed");

		ret = bt_r_and_c_discover(msg->conn);
		if (ret) {
			LOG_WRN("Failed to discover rendering services");
		}

		if (IS_ENABLED(CONFIG_STREAM_BIDIRECTIONAL)) {
			ret = unicast_client_discover(msg->conn, UNICAST_SERVER_SOURCE);
		} else {
			ret = unicast_client_discover(msg->conn, UNICAST_SERVER_SINK);
		}

		if (ret) {
			LOG_ERR("Failed to handle unicast client discover: %d", ret);
		}

		break;

	case BT_MGMT_DISCONNECTED:
		/* NOTE: The string below is used by the Nordic CI system */
		LOG_INF("Disconnection event. Num connections: %u", num_conn);

		unicast_client_conn_disconnected(msg->conn);
		break;

	default:
		LOG_WRN("Unexpected/unhandled bt_mgmt event: %d", msg->event);
		break;
	}
}

ZBUS_LISTENER_DEFINE(bt_mgmt_evt_listen, bt_mgmt_evt_handler);

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

	content_control_thread_id = k_thread_create(
		&content_control_msg_sub_thread_data, content_control_msg_sub_thread_stack,
		CONFIG_CONTENT_CONTROL_MSG_SUB_STACK_SIZE,
		(k_thread_entry_t)content_control_msg_sub_thread, NULL, NULL, NULL,
		K_PRIO_PREEMPT(CONFIG_CONTENT_CONTROL_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(content_control_thread_id, "CONTENT_CONTROL_MSG_SUB");
	if (ret) {
		return ret;
	}

	return 0;
}

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

	ret = zbus_chan_add_obs(&cont_media_chan, &content_control_evt_sub,
				ZBUS_ADD_OBS_TIMEOUT_MS);

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
		ret = unicast_client_send(0, enc_audio);

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

#include <zephyr/bluetooth/hci.h>
#include <bluetooth/hci_vs_sdc.h>
#include "chmap_filter.h"

struct params_ble {
	uint16_t sample_count_min;
	uint8_t min_channel_count;
	int16_t weight_crc_ok;
	int16_t weight_crc_error;
	uint16_t ble_block_threshold;
	uint8_t eval_max_count;
	uint16_t eval_duration;
	uint16_t eval_keepout_duration;
	uint16_t eval_success_threshold;
} __packed;

struct params_wifi {
	int16_t wifi_rating_inc;
	int16_t wifi_present_threshold;
	int16_t wifi_active_threshold;
} __packed;

struct params_chmap {
	uint8_t chmap[CHMAP_BLE_BITMASK_SIZE];
} __packed;

struct params_blacklist {
	uint16_t wifi_chn_bitmask;
} __packed;

static uint8_t chmap_instance_buf[CHMAP_FILTER_INST_SIZE] __aligned(CHMAP_FILTER_INST_ALIGN);
static struct chmap_instance *chmap_inst;
static uint8_t current_chmap[CHMAP_BLE_BITMASK_SIZE] = CHMAP_BLE_BITMASK_DEFAULT;
static atomic_t processing;
static atomic_t new_blacklist;
static atomic_t params_updated;
static struct chmap_filter_params filter_params;
static struct k_mutex data_access_mutex;
static struct chmap_instance *chmap_inst;
#define INVALID_BLACKLIST 0xFFFF


static bool on_vs_evt(struct net_buf_simple *buf)
{
	uint8_t *subevent_code;
	sdc_hci_subevent_vs_qos_conn_event_report_t *evt;

	subevent_code = net_buf_simple_pull_mem(
		buf,
		sizeof(*subevent_code));

	switch (*subevent_code) {
	case SDC_HCI_SUBEVENT_VS_QOS_CONN_EVENT_REPORT:
		evt = (void *)buf->data;
		//LOG_INF("conn_handle: %2d, evt = %6d, ch_index: %2d, crc_ok: %d, crc_err: %d, crc_nak: %d",
		//	evt->conn_handle, evt->event_counter, evt->channel_index, evt->crc_ok_count, evt->crc_error_count, evt->nak_count);
		if (evt->crc_error_count > 0) {
			LOG_WRN("ch_index %d CRC error: %d", evt->channel_index, evt->crc_error_count);
		}
		chmap_filter_crc_update(
			chmap_inst,
			evt->channel_index,
			evt->crc_ok_count,
			evt->crc_error_count);
		return true;
	default:
		return false;
	}
}

static void enable_qos_reporting(void)
{
	int err;
	sdc_hci_cmd_vs_qos_conn_event_report_enable_t cmd_enable;

	err = bt_hci_register_vnd_evt_cb(on_vs_evt);
	if (err) {
		LOG_ERR("Failed to register HCI VS callback");
		return;
	}

	cmd_enable.enable = 1;

	err = hci_vs_sdc_qos_conn_event_report_enable(&cmd_enable);
	if (err) {
		LOG_ERR("Failed to enable HCI VS QoS");
	}
}

static void apply_new_params(void)
{
	int err;

	k_mutex_lock(&data_access_mutex, K_FOREVER);
	/* chmap_filter_params_set returns immediately */
	err = chmap_filter_params_set(chmap_inst, &filter_params);
	atomic_set(&params_updated, false);
	k_mutex_unlock(&data_access_mutex);

	if (err) {
		LOG_WRN("Param update failed");
	}
}

static void ble_qos_thread_fn(void)
{
	int err;
	bool update_channel_map;
	uint8_t *chmap;

	while (true) {

		//Configure processing interval for QoS algorithm.
		k_sleep(K_MSEC(1000));

		/* Check and apply new parameters received via config channel */
		if (atomic_get(&params_updated)) {
			LOG_INF("Applying new parameters");
			apply_new_params();
		}

		/* Check and apply new blacklist received via config channel */
		uint16_t blacklist_update =
			(uint16_t) atomic_set(&new_blacklist, INVALID_BLACKLIST);

		if (blacklist_update != INVALID_BLACKLIST) {
			err = chmap_filter_blacklist_set(
				chmap_inst,
				blacklist_update);
			if (err) {
				LOG_WRN("Blacklist update failed");
			}
		}

		/* Run processing function. */
		/* Atomic variable is used as data busy flag */
		/* (this thread runs at the lowest priority) */
		atomic_set(&processing, true);
		update_channel_map = chmap_filter_process(chmap_inst);
		atomic_set(&processing, false);

		if (!update_channel_map) {
			continue;
		}

		chmap = chmap_filter_suggested_map_get(chmap_inst);

		err = bt_le_set_chan_map(chmap);
		if (err) {
			LOG_WRN("bt_le_set_chan_map: %d", err);
		} else {
			LOG_WRN("Channel map update");
			LOG_WRN("chmap: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
				chmap[0], chmap[1], chmap[2], chmap[3], chmap[4]);
		}

		chmap_filter_suggested_map_confirm(chmap_inst);
		k_mutex_lock(&data_access_mutex, K_FOREVER);
		memcpy(current_chmap, chmap, sizeof(current_chmap));
		k_mutex_unlock(&data_access_mutex);
	}
}

static void chmap_filter_setup(void)
{
	int ret;

	chmap_filter_init();

	chmap_inst =
		(struct chmap_instance *) chmap_instance_buf;
	ret = chmap_filter_instance_init(
		chmap_inst,
		sizeof(chmap_instance_buf));
	if (ret) {
		LOG_ERR("Failed to initialize filter, ret: %d", ret);
		return;
	}
	LOG_INF("Chmap lib version: %s",
		chmap_filter_version());

	chmap_filter_params_get(chmap_inst, &filter_params);

	k_mutex_init(&data_access_mutex);
	new_blacklist = INVALID_BLACKLIST;
	atomic_set(&params_updated, false);

	ble_qos_thread_id = k_thread_create(&ble_qos_thread_data, ble_qos_thread_stack,
			BLE_QOS_THREAD_STACK_SIZE,
			(k_thread_entry_t)ble_qos_thread_fn,
			NULL, NULL, NULL,
			K_PRIO_PREEMPT(K_LOWEST_APPLICATION_THREAD_PRIO), 0, K_NO_WAIT);

	ret = k_thread_name_set(ble_qos_thread_id, "BLE_QOS");
	if (ret) {
		LOG_ERR("Failed to create ble_qos thread");
		return;
	}

	enable_qos_reporting();
}

int main(void)
{
	int ret;

	LOG_DBG("Main started");

	ret = nrf5340_audio_dk_init();
	ERR_CHK(ret);

	ret = fw_info_app_print();
	ERR_CHK(ret);

	ret = bt_mgmt_init();
	ERR_CHK(ret);

	ret = audio_system_init();
	ERR_CHK(ret);

	ret = zbus_subscribers_create();
	ERR_CHK_MSG(ret, "Failed to create zbus subscriber threads");

	ret = zbus_link_producers_observers();
	ERR_CHK_MSG(ret, "Failed to link zbus producers and observers");

	ret = le_audio_rx_init();
	ERR_CHK(ret);

	ret = bt_r_and_c_init();
	ERR_CHK(ret);

	ret = bt_content_ctrl_init();
	ERR_CHK(ret);

	ret = unicast_client_enable(0, le_audio_rx_data_handler);
	ERR_CHK(ret);

	chmap_filter_setup();

	ret = bt_mgmt_scan_start(0, 0, BT_MGMT_SCAN_TYPE_CONN, CONFIG_BT_DEVICE_NAME,
				 BRDCAST_ID_NOT_USED);
	if (ret) {
		LOG_ERR("Failed to start scanning");
		return ret;
	}

	return 0;
}
