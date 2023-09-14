/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "streamctrl.h"

#include <zephyr/debug/stack.h>
#include <zephyr/zbus/zbus.h>

#include "nrf5340_audio_common.h"
#include "led.h"
#include "button_assignments.h"
#include "macros_common.h"
#include "audio_system.h"
#include "button_handler.h"
#include "data_fifo.h"
#include "le_audio.h"
#include "bt_mgmt.h"
#include "bt_rend.h"
#include "audio_datapath.h"
#include "bt_content_ctrl.h"
#include "unicast_client.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(streamctrl_unicast_client, CONFIG_STREAMCTRL_LOG_LEVEL);

static enum stream_state strm_state = STATE_PAUSED;

struct ble_iso_data {
	uint8_t data[CONFIG_BT_ISO_RX_MTU];
	size_t data_size;
	bool bad_frame;
	uint32_t sdu_ref;
	uint32_t recv_frame_ts;
} __packed;

struct rx_stats {
	uint32_t recv_cnt;
	uint32_t bad_frame_cnt;
	uint32_t data_size_mismatch_cnt;
};

DATA_FIFO_DEFINE(ble_fifo_rx, CONFIG_BUF_BLE_RX_PACKET_NUM, WB_UP(sizeof(struct ble_iso_data)));

ZBUS_SUBSCRIBER_DEFINE(button_evt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);
ZBUS_SUBSCRIBER_DEFINE(le_audio_evt_sub, CONFIG_LE_AUDIO_MSG_SUB_QUEUE_SIZE);
ZBUS_SUBSCRIBER_DEFINE(content_control_evt_sub, CONFIG_CONTENT_CONTROL_MSG_SUB_QUEUE_SIZE);

ZBUS_CHAN_DECLARE(button_chan);
ZBUS_CHAN_DECLARE(le_audio_chan);
ZBUS_CHAN_DECLARE(bt_mgmt_chan);
ZBUS_CHAN_DECLARE(cont_media_chan);
ZBUS_CHAN_DECLARE(sdu_ref_chan);

ZBUS_OBS_DECLARE(sdu_ref_msg_listen);

static struct k_thread audio_datapath_thread_data;
static struct k_thread button_msg_sub_thread_data;
static struct k_thread le_audio_msg_sub_thread_data;
static struct k_thread content_control_msg_sub_thread_data;

static k_tid_t audio_datapath_thread_id;
static k_tid_t button_msg_sub_thread_id;
static k_tid_t le_audio_msg_sub_thread_id;
static k_tid_t content_control_thread_id;

K_THREAD_STACK_DEFINE(audio_datapath_thread_stack, CONFIG_AUDIO_DATAPATH_STACK_SIZE);
K_THREAD_STACK_DEFINE(button_msg_sub_thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE);
K_THREAD_STACK_DEFINE(le_audio_msg_sub_thread_stack, CONFIG_LE_AUDIO_MSG_SUB_STACK_SIZE);
K_THREAD_STACK_DEFINE(content_control_msg_sub_thread_stack,
		      CONFIG_CONTENT_CONTROL_MSG_SUB_STACK_SIZE);

static void bt_mgmt_evt_handler(const struct zbus_channel *chan);
ZBUS_LISTENER_DEFINE(bt_mgmt_evt_listen, bt_mgmt_evt_handler);

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
		if (msg.event == MEDIA_PLAY) {
			unicast_client_start();
		} else {
			unicast_client_stop();
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

		default:
			LOG_WRN("Unexpected/unhandled button id: %d", msg.button_pin);
		}

		STACK_USAGE_PRINT("button_msg_thread", &button_msg_sub_thread_data);
	}
}

/* Callback for handling BLE RX */
static void le_audio_rx_data_handler(uint8_t const *const p_data, size_t data_size, bool bad_frame,
				     uint32_t sdu_ref, enum audio_channel channel_index,
				     size_t desired_data_size)
{
	/* Capture timestamp of when audio frame is received */
	uint32_t recv_frame_ts = nrfx_timer_capture(&audio_sync_timer_instance,
						    AUDIO_SYNC_TIMER_CURR_TIME_CAPTURE_CHANNEL);

	/* Since the audio datapath thread is preemptive, no actions on the
	 * FIFO can happen whilst in this handler.
	 */

	bool data_size_mismatch = false;
	static struct rx_stats rx_stats[AUDIO_CH_NUM];

	rx_stats[channel_index].recv_cnt++;
	if (data_size != desired_data_size && !bad_frame) {
		data_size_mismatch = true;
		rx_stats[channel_index].data_size_mismatch_cnt++;
	}

	if (bad_frame) {
		rx_stats[channel_index].bad_frame_cnt++;
	}

	if ((rx_stats[channel_index].recv_cnt % 100) == 0 && rx_stats[channel_index].recv_cnt) {
		/* NOTE: The string below is used by the Nordic CI system */
		LOG_DBG("ISO RX SDUs: Ch: %d Total: %d Bad: %d Size mismatch %d", channel_index,
			rx_stats[channel_index].recv_cnt, rx_stats[channel_index].bad_frame_cnt,
			rx_stats[channel_index].data_size_mismatch_cnt);
	}

	if (data_size_mismatch) {
		/* Return if sizes do not match */
		return;
	}

	if (strm_state != STATE_STREAMING) {
		/* Throw away data */
		LOG_DBG("Not in streaming state, throwing data: %d", strm_state);
		return;
	}

	int ret;
	struct ble_iso_data *iso_received = NULL;

#if (CONFIG_AUDIO_DEV == GATEWAY)
	if (channel_index != AUDIO_CH_L) {
		/* Only left channel RX data in use on gateway */
		return;
	}
#endif /* (CONFIG_AUDIO_DEV == GATEWAY) */

	uint32_t blocks_alloced_num, blocks_locked_num;

	ret = data_fifo_num_used_get(&ble_fifo_rx, &blocks_alloced_num, &blocks_locked_num);
	ERR_CHK(ret);

	if (blocks_alloced_num >= CONFIG_BUF_BLE_RX_PACKET_NUM) {
		/* FIFO buffer is full, swap out oldest frame for a new one */

		void *stale_data;
		size_t stale_size;

		LOG_WRN("BLE ISO RX overrun");

		ret = data_fifo_pointer_last_filled_get(&ble_fifo_rx, &stale_data, &stale_size,
							K_NO_WAIT);
		ERR_CHK(ret);

		data_fifo_block_free(&ble_fifo_rx, &stale_data);
	}

	ret = data_fifo_pointer_first_vacant_get(&ble_fifo_rx, (void *)&iso_received, K_NO_WAIT);
	ERR_CHK_MSG(ret, "Unable to get FIFO pointer");

	if (data_size > ARRAY_SIZE(iso_received->data)) {
		ERR_CHK_MSG(-ENOMEM, "Data size too large for buffer");
		return;
	}

	memcpy(iso_received->data, p_data, data_size);

	iso_received->bad_frame = bad_frame;
	iso_received->data_size = data_size;
	iso_received->sdu_ref = sdu_ref;
	iso_received->recv_frame_ts = recv_frame_ts;

	ret = data_fifo_block_lock(&ble_fifo_rx, (void *)&iso_received,
				   sizeof(struct ble_iso_data));
	ERR_CHK_MSG(ret, "Failed to lock block");
}

/* Thread to receive data from BLE through a k_fifo and send to audio datapath */
static void audio_datapath_thread(void *dummy1, void *dummy2, void *dummy3)
{
	int ret;
	struct ble_iso_data *iso_received = NULL;
	size_t iso_received_size;

	while (1) {
		ret = data_fifo_pointer_last_filled_get(&ble_fifo_rx, (void *)&iso_received,
							&iso_received_size, K_FOREVER);
		ERR_CHK(ret);

#if ((CONFIG_AUDIO_DEV == GATEWAY) && (CONFIG_AUDIO_SOURCE_USB))
		ret = audio_decode(iso_received->data, iso_received->data_size,
				   iso_received->bad_frame);
		ERR_CHK(ret);
#else
		audio_datapath_stream_out(iso_received->data, iso_received->data_size,
					  iso_received->sdu_ref, iso_received->bad_frame,
					  iso_received->recv_frame_ts);
#endif /* ((CONFIG_AUDIO_DEV == GATEWAY) && (CONFIG_AUDIO_SOURCE_USB)) */
		data_fifo_block_free(&ble_fifo_rx, (void *)&iso_received);

		STACK_USAGE_PRINT("audio_datapath_thread", &audio_datapath_thread_data);
	}
}

/* Function for handling all stream state changes */
static void stream_state_set(enum stream_state stream_state_new)
{
	strm_state = stream_state_new;
}

uint8_t stream_state_get(void)
{
	return strm_state;
}

/**
 * @brief	Handle Bluetooth LE audio events.
 */
static void le_audio_msg_sub_thread(void)
{
	int ret;
	const struct zbus_channel *chan;

	while (1) {
		ret = zbus_sub_wait(&le_audio_evt_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		struct le_audio_msg msg;

		ret = zbus_chan_read(chan, &msg, ZBUS_READ_TIMEOUT_MS);
		ERR_CHK(ret);

		uint8_t event = msg.event;

		LOG_DBG("Received event = %d, current state = %d", event, strm_state);

		switch (event) {
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

		default:
			LOG_WRN("Unexpected/unhandled le_audio event: %d", event);

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

	audio_datapath_thread_id = k_thread_create(
		&audio_datapath_thread_data, audio_datapath_thread_stack,
		CONFIG_AUDIO_DATAPATH_STACK_SIZE, (k_thread_entry_t)audio_datapath_thread, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_AUDIO_DATAPATH_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(audio_datapath_thread_id, "AUDIO DATAPATH");
	if (ret) {
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

	ret = zbus_chan_add_obs(&sdu_ref_chan, &sdu_ref_msg_listen, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add timestamp listener");
		return ret;
	}

	return 0;
}

/**
 * @brief	Zbus listener to receive events from bt_mgmt.
 *
 * @param[in] chan	Zbus channel.
 *
 * @note	Will in most cases be called from BT_RX context,
 *		so there should not be too much processing done here.
 */
static void bt_mgmt_evt_handler(const struct zbus_channel *chan)
{
	int ret;
	const struct bt_mgmt_msg *msg;

	msg = zbus_chan_const_msg(chan);
	uint8_t event = msg->event;

	switch (event) {
	case BT_MGMT_CONNECTED:
		LOG_INF("Device connected");
		break;
	case BT_MGMT_SECURITY_CHANGED:
		LOG_INF("Security changed");

		ret = bt_rend_discover(msg->conn);
		if (ret) {
			LOG_WRN("Failed to discover rendering services");
		}
#if (CONFIG_STREAM_BIDIRECTIONAL)
		ret = unicast_client_discover(msg->conn, UNICAST_SERVER_BIDIR);
#else
		ret = unicast_client_discover(msg->conn, UNICAST_SERVER_SOURCE);
#endif
		if (ret) {
			LOG_ERR("Failed to handle unicast client discover: %d", ret);
		}
		break;
	case BT_MGMT_DISCONNECTED:
		LOG_INF("Device disconnected");
		unicast_client_conn_disconnected(msg->conn);
		break;
	default:
		LOG_WRN("Unexpected/unhandled bt_mgmt event: %d", event);
		break;
	}
}

/**
 * @brief	Link zbus producers and observers.
 *
 * @return	0 for success, error otherwise.
 */
static int zbus_link_producers_observers(void)
{
	int ret;

	if (!IS_ENABLED(CONFIG_ZBUS) || (CONFIG_ZBUS_RUNTIME_OBSERVERS_POOL_SIZE <= 0)) {
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

void streamctrl_send(void const *const data, size_t size, uint8_t num_ch)
{
	int ret;
	static int prev_ret;

	struct encoded_audio enc_audio = {.data = data, .size = size, .num_ch = num_ch};

	if (strm_state == STATE_STREAMING) {
		ret = unicast_client_send(enc_audio);

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

int streamctrl_start(void)
{
	int ret;
	static bool started;

	if (started) {
		LOG_WRN("Streamctrl already started");
		return -EALREADY;
	}

	ret = data_fifo_init(&ble_fifo_rx);
	ERR_CHK_MSG(ret, "Failed to set up ble_rx FIFO");

	ret = audio_system_init();
	ERR_CHK_MSG(ret, "Failed to initialize the audio system");

	ret = zbus_subscribers_create();
	ERR_CHK_MSG(ret, "Failed to create zbus subscriber threads");

	ret = zbus_link_producers_observers();
	ERR_CHK_MSG(ret, "Failed to link zbus producers and observers");

	ret = bt_rend_init();
	ERR_CHK(ret);

	ret = bt_content_ctrl_init();
	ERR_CHK(ret);

	ret = unicast_client_enable(le_audio_rx_data_handler);
	ERR_CHK(ret);

	started = true;

	ret = bt_mgmt_scan_start(0, 0, BT_MGMT_SCAN_TYPE_CONN, CONFIG_BT_DEVICE_NAME);
	return 0;
}
