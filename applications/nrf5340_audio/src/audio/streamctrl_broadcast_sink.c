/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "streamctrl.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#include "nrf5340_audio_common.h"
#include "broadcast_sink.h"
#include "led.h"
#include "button_assignments.h"
#include "macros_common.h"
#include "audio_system.h"
#include "bt_mgmt.h"
#include "bt_rend.h"
#include "audio_datapath.h"
#include "le_audio_rx.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(streamctrl_broadcast_sink, CONFIG_STREAMCTRL_LOG_LEVEL);

struct ble_iso_data {
	uint8_t data[CONFIG_BT_ISO_RX_MTU];
	size_t data_size;
	bool bad_frame;
	uint32_t sdu_ref;
	uint32_t recv_frame_ts;
} __packed;

ZBUS_SUBSCRIBER_DEFINE(button_evt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);

ZBUS_MSG_SUBSCRIBER_DEFINE(le_audio_evt_sub);

ZBUS_CHAN_DECLARE(button_chan);
ZBUS_CHAN_DECLARE(le_audio_chan);
ZBUS_CHAN_DECLARE(bt_mgmt_chan);
ZBUS_CHAN_DECLARE(volume_chan);

ZBUS_OBS_DECLARE(volume_evt_sub);

static struct k_thread button_msg_sub_thread_data;
static struct k_thread le_audio_msg_sub_thread_data;

static k_tid_t button_msg_sub_thread_id;
static k_tid_t le_audio_msg_sub_thread_id;

K_THREAD_STACK_DEFINE(button_msg_sub_thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE);
K_THREAD_STACK_DEFINE(le_audio_msg_sub_thread_stack, CONFIG_LE_AUDIO_MSG_SUB_STACK_SIZE);

static enum stream_state strm_state = STATE_PAUSED;

static uint32_t current_broadcast_id = 0xffffff;

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/sys/byteorder.h>
static struct bt_le_scan_cb scan_callback;
#define INVALID_BROADCAST_ID 0xFFFFFFFF
struct broadcast_source {
	char name[BLE_SEARCH_NAME_MAX_LEN];
	uint32_t broadcast_id;
	bool high_pri_stream;
};

static bool scan_check_high_pri_audio(struct bt_data *data, void *user_data)
{
	struct broadcast_source *source = (struct broadcast_source *)user_data;
	//bool *found_high_pri_stream = (bool *)user_data;
	struct bt_uuid_16 adv_uuid;
	int i;
	bool stream_context_found;
	switch (data->type) {
	case BT_DATA_SVC_DATA16:
	case BT_DATA_UUID16_SOME:
	case BT_DATA_UUID16_ALL:
		for (i = 0; i < data->data_len; i += sizeof(uint16_t)) {
			struct bt_uuid *uuid;
			struct bt_le_conn_param *param;
			uint16_t u16;
			int err;

			memcpy(&u16, &data->data[i], sizeof(u16));
			uuid = BT_UUID_DECLARE_16(sys_le16_to_cpu(u16));
			if (bt_uuid_cmp(uuid, BT_UUID_PBA) == 0) {
				LOG_HEXDUMP_INF(data->data, data->data_len, "");
				if (data->data[3] > 0) {
					if (data->data[10] == 4){
						//LOG_WRN("Found high pri stream");
						source->high_pri_stream = true;
					}
				}
			} else if (bt_uuid_cmp(uuid, BT_UUID_BROADCAST_AUDIO) == 0){
				//LOG_HEXDUMP_INF(data->data, data->data_len, "audio broadcast");
				source->broadcast_id = sys_get_le24(data->data + BT_UUID_SIZE_16);
				LOG_WRN("found broadcast id %x", source->broadcast_id);
			} 
		}
	}
	return true;
}
static struct bt_le_scan_recv_info store_info;
static uint32_t store_broadcast_id;
#include "bt_mgmt_scan_for_broadcast_internal.h"
static void scan_recv_cb(const struct bt_le_scan_recv_info *info, struct net_buf_simple *ad)
{
	struct broadcast_source source = {.broadcast_id = INVALID_BROADCAST_ID};
	int ret;
	/* We are only interested in non-connectable periodic advertisers */
	if ((info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) || info->interval == 0) {
		return;
	}

	bt_data_parse(ad, scan_check_high_pri_audio, (void *)&source);
	if(source.high_pri_stream) {
		if (source.broadcast_id != current_broadcast_id) {
			LOG_ERR("should switch stream");
			bt_le_scan_cb_unregister(&scan_callback);
			bt_le_scan_stop();
			struct bt_mgmt_msg msg;
			msg.event = BT_MGMT_SWITCH;
			store_broadcast_id = source.broadcast_id;
			//current_broadcast_id = source.broadcast_id;
			memcpy(&store_info, info, sizeof(store_info));

			ret = zbus_chan_pub(&bt_mgmt_chan, &msg, K_NO_WAIT);
			ERR_CHK(ret);
		} else {
			LOG_ERR("same stream, no need to switch");
		}
	}
}



void scan_for_high_pri_stream()
{
	static int scan_interval = CONFIG_BT_BACKGROUND_SCAN_INTERVAL;
	static int scan_window = CONFIG_BT_BACKGROUND_SCAN_WINDOW;
	struct bt_le_scan_param *scan_param =
		BT_LE_SCAN_PARAM(NRF5340_AUDIO_GATEWAY_SCAN_TYPE, BT_LE_SCAN_OPT_FILTER_DUPLICATE,
				 scan_interval, scan_window);

	scan_callback.recv = scan_recv_cb;
	bt_le_scan_cb_register(&scan_callback);
	bt_le_scan_start(scan_param, NULL);
}


/* Function for handling all stream state changes */
static void stream_state_set(enum stream_state stream_state_new)
{
	strm_state = stream_state_new;
}

/**
 * @brief	Handle button activity.
 */
static void button_msg_sub_thread(void)
{
	int ret;
	const struct zbus_channel *chan;
	bool broadcast_alt = true;

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
				ret = broadcast_sink_stop();
				if (ret) {
					LOG_WRN("Failed to stop broadcast sink: %d", ret);
				}
			} else if (strm_state == STATE_PAUSED) {
				ret = broadcast_sink_start();
				if (ret) {
					LOG_WRN("Failed to start broadcast sink: %d", ret);
				}
			} else {
				LOG_WRN("In invalid state: %d", strm_state);
			}

			break;

		case BUTTON_VOLUME_UP:
			ret = bt_rend_volume_up();
			if (ret) {
				LOG_WRN("Failed to increase volume: %d", ret);
			}

			break;

		case BUTTON_VOLUME_DOWN:
			ret = bt_rend_volume_down();
			if (ret) {
				LOG_WRN("Failed to decrease volume: %d", ret);
			}

			break;

		case BUTTON_4:
			ret = broadcast_sink_change_active_audio_stream();
			if (ret) {
				LOG_WRN("Failed to change active audio stream: %d", ret);
			}

			break;

		case BUTTON_5:
			if (IS_ENABLED(CONFIG_AUDIO_MUTE)) {
				ret = bt_rend_volume_mute(false);
				if (ret) {
					LOG_WRN("Failed to mute, ret: %d", ret);
				}

				break;
			}

			ret = broadcast_sink_disable();
			if (ret) {
				LOG_ERR("Failed to disable the broadcast sink: %d", ret);
				break;
			}

			if (broadcast_alt) {
				ret = bt_mgmt_scan_start(0, 0, BT_MGMT_SCAN_TYPE_BROADCAST,
							 CONFIG_BT_AUDIO_BROADCAST_NAME_ALT);
				broadcast_alt = false;
			} else {
				ret = bt_mgmt_scan_start(0, 0, BT_MGMT_SCAN_TYPE_BROADCAST,
							 CONFIG_BT_AUDIO_BROADCAST_NAME);
				broadcast_alt = true;
			}

			if (ret) {
				LOG_WRN("Failed to start scanning for broadcaster: %d", ret);
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
	uint32_t pres_delay_us;
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

			audio_system_start();
			stream_state_set(STATE_STREAMING);
			ret = led_blink(LED_APP_1_BLUE);
			ERR_CHK(ret);
			scan_for_high_pri_stream();
			break;

		case LE_AUDIO_EVT_NOT_STREAMING:
			LOG_DBG("LE audio evt not_streaming");

			if (strm_state == STATE_PAUSED) {
				LOG_DBG("Got not_streaming event in paused state");
				break;
			}

			stream_state_set(STATE_PAUSED);
			audio_system_stop();
			ret = led_on(LED_APP_1_BLUE);
			ERR_CHK(ret);

			break;

		case LE_AUDIO_EVT_CONFIG_RECEIVED:
			LOG_DBG("Config received");

			ret = broadcast_sink_config_get(&bitrate_bps, &sampling_rate_hz,
							&pres_delay_us);
			if (ret) {
				LOG_WRN("Failed to get config: %d", ret);
				break;
			}

			LOG_DBG("Sampling rate: %d Hz", sampling_rate_hz);
			LOG_DBG("Bitrate: %d bps", bitrate_bps);

			ret = audio_datapath_pres_delay_us_set(pres_delay_us);
			if (ret) {
				LOG_ERR("Failed to set presentation delay to %d", pres_delay_us);
				break;
			}

			LOG_INF("Presentation delay %d us is set", pres_delay_us);

			break;

		case LE_AUDIO_EVT_SYNC_LOST:
			LOG_INF("Sync lost");

			ret = bt_mgmt_pa_sync_delete(msg.pa_sync);
			if (ret) {
				LOG_WRN("Failed to delete PA sync");
			}

			if (strm_state == STATE_STREAMING) {
				stream_state_set(STATE_PAUSED);
				audio_system_stop();
				ret = led_on(LED_APP_1_BLUE);
				ERR_CHK(ret);
			}

			if (IS_ENABLED(CONFIG_BT_OBSERVER)) {
				ret = bt_mgmt_scan_start(0, 0, BT_MGMT_SCAN_TYPE_BROADCAST, NULL);
				if (ret) {
					if (ret == -EALREADY) {
						return;
					}

					LOG_ERR("Failed to restart scanning: %d", ret);
					break;
				}

				/* NOTE: The string below is used by the Nordic CI system */
				LOG_INF("Restarted scanning for broadcaster");
			}

			break;

		case LE_AUDIO_EVT_NO_VALID_CFG:
			LOG_WRN("No valid configurations found, disabling the broadcast sink");

			ret = broadcast_sink_disable();
			if (ret) {
				LOG_ERR("Failed to disable the broadcast sink: %d", ret);
				break;
			}

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

	return 0;
}


static void pa_sync_worker(struct k_work *work)
{
	int ret;
	LOG_WRN("target broadcast id = %x, pass to periodic_adv_sync", store_broadcast_id);
	periodic_adv_sync(&store_info, store_broadcast_id);
}

K_WORK_DEFINE(pa_sync_work, pa_sync_worker);

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
	case BT_MGMT_PA_SYNCED:
		LOG_INF("PA synced, id = %X", msg->broadcast_id);

		ret = broadcast_sink_pa_sync_set(msg->pa_sync, msg->broadcast_id);
		if (ret) {
			LOG_WRN("Failed to set PA sync");
			ret = bt_mgmt_scan_start(0, 0, BT_MGMT_SCAN_TYPE_BROADCAST, NULL);
			if (ret) {
				if (ret == -EALREADY) {
					return;
				}

				LOG_ERR("Failed to restart scanning: %d", ret);
			}
		}else {
			current_broadcast_id = msg->broadcast_id;
			LOG_WRN("BT_MGMT_PA_SYNCED, current id = %x", msg->broadcast_id);
		}

		break;

	case BT_MGMT_PA_SYNC_LOST:
		LOG_INF("PA sync lost, reason: %d", msg->pa_sync_term_reason);
		bt_le_scan_cb_unregister(&scan_callback);
		bt_le_scan_stop();
		current_broadcast_id = 0xffffff;
		if (IS_ENABLED(CONFIG_BT_OBSERVER) &&
			msg->pa_sync_term_reason != BT_HCI_ERR_LOCALHOST_TERM_CONN) {
			LOG_WRN("BT_MGMT_PA_SYNC_LOST trigger scan");
			ret = bt_mgmt_scan_start(0, 0, BT_MGMT_SCAN_TYPE_BROADCAST, NULL);
			if (ret) {
				if (ret == -EALREADY) {
					return;
				}

				LOG_ERR("Failed to restart scanning: %d", ret);
				break;
			}

			/* NOTE: The string below is used by the Nordic CI system */
			LOG_INF("Restarted scanning for broadcaster");
		}

		break;

		case BT_MGMT_SWITCH:
			LOG_WRN("BT_MGMT_SWITCH");
			ret = broadcast_sink_disable();
			if (ret) {
				LOG_ERR("Failed to disable the broadcast sink: %d", ret);
				break;
			}
			k_work_submit(&pa_sync_work);
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

	ret = zbus_chan_add_obs(&volume_chan, &volume_evt_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add add volume sub");
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
	ARG_UNUSED(data);
	ARG_UNUSED(size);
	ARG_UNUSED(num_ch);

	LOG_WRN("Sending is not possible for broadcast sink");
}

int streamctrl_start(void)
{
	int ret;
	static bool started;

	if (started) {
		LOG_WRN("Streamctrl already started");
		return -EALREADY;
	}

	ret = audio_system_init();
	ERR_CHK_MSG(ret, "Failed to initialize the audio system");

	ret = zbus_subscribers_create();
	ERR_CHK_MSG(ret, "Failed to create zbus subscriber threads");

	ret = zbus_link_producers_observers();
	ERR_CHK_MSG(ret, "Failed to link zbus producers and observers");

	ret = le_audio_rx_init();
	ERR_CHK_MSG(ret, "Failed to initialize rx path");

	ret = broadcast_sink_enable(le_audio_rx_data_handler);
	ERR_CHK_MSG(ret, "Failed to enable broadcast sink");

	ret = bt_mgmt_scan_start(0, 0, BT_MGMT_SCAN_TYPE_BROADCAST, CONFIG_BT_AUDIO_BROADCAST_NAME);
	ERR_CHK_MSG(ret, "Failed to start scanning");

	started = true;

	return 0;
}
