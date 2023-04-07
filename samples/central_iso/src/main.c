/*
 * Copyright (c) 2021 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <nrfx_timer.h>
#include <nrfx_clock.h>
#include <zephyr/init.h>
#include <nrfx_dppi.h>
#include <nrfx_ipc.h>
#include <dk_buttons_and_leds.h>
LOG_MODULE_REGISTER(ble);

static void start_scan(void);

#define DEVICE_NAME_PEER_L "sync_demo_L"
#define DEVICE_NAME_PEER_R "sync_demo_R"
#define DEVICE_NAME_PEER_L_LEN (sizeof(DEVICE_NAME_PEER_L) - 1)
#define DEVICE_NAME_PEER_R_LEN (sizeof(DEVICE_NAME_PEER_R) - 1)
#define CIS_CONN_RETRY_TIMES 5
#define ISO_MAX_PAYLOAD_TX 5
#define ISO_MAX_PAYLOAD_RX 30
#define ISO_RTN_TIMES 1
#define BT_LE_SCAN_PASSIVE_INTENSIVE                                                               \
	BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_PASSIVE, BT_LE_SCAN_OPT_FILTER_DUPLICATE, 0x10,           \
			 0x10) /* set scan window and interval to 10ms*/

#define BT_LE_CONN_PARAM_MULTI BT_LE_CONN_PARAM(80, 80, 0, 400) /* set ACL interval to 100ms*/
#define ISO_SYNC_TIMER_CURR_TIME_CAPTURE_CHANNEL 1
#define ISO_SYNC_TIMER_TIMER_TRIGGER_CHANNEL 0
#define LED_TOGGLE_INTERVAL_US (1000000) /* 1 s */
#define PRESENTATION_DELAY_US (20000) /*20 ms*/
#define ISO_INTERVAL_US (5000) /* 5ms */
#define SELF_OFFSET_US (1302)
//const nrfx_timer_t iso_sync_timer_instance;

static struct bt_conn *gateway_conn_peer[CONFIG_BT_MAX_CONN];
static struct k_work iso_send_work;

static struct bt_iso_chan iso_chan[CONFIG_BT_ISO_MAX_CHAN];
static struct bt_iso_chan *iso_chan_p[CONFIG_BT_ISO_MAX_CHAN];
static struct k_work_delayable iso_cis_conn_work;

#define HCI_ISO_BUF_ALLOC_PER_CHAN 1
NET_BUF_POOL_FIXED_DEFINE(tx_pool_l, 1, BT_ISO_SDU_BUF_SIZE(ISO_MAX_PAYLOAD_TX), 8, NULL);
NET_BUF_POOL_FIXED_DEFINE(tx_pool_r, 1, BT_ISO_SDU_BUF_SIZE(ISO_MAX_PAYLOAD_TX), 8, NULL);

#define ISO_SYNC_TIMER_INSTANCE_NUMBER 1
#define LED_TOGGLING_TIMER_INSTANCE_NUMBER 2
#define ISO_SYNC_TIMER_NET_APP_IPC_EVT NRF_IPC_EVENT_RECEIVE_4

const nrfx_timer_t iso_sync_timer_instance = NRFX_TIMER_INSTANCE(ISO_SYNC_TIMER_INSTANCE_NUMBER);
const nrfx_timer_t led_toggling_timer_instance =
	NRFX_TIMER_INSTANCE(LED_TOGGLING_TIMER_INSTANCE_NUMBER);

static uint8_t dppi_channel_timer_clear;

static nrfx_timer_config_t cfg = { .frequency = NRF_TIMER_FREQ_1MHz,
				   .mode = NRF_TIMER_MODE_TIMER,
				   .bit_width = NRF_TIMER_BIT_WIDTH_32,
				   .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
				   .p_context = NULL };

atomic_t iso_tx_pool_alloc[2];

static uint32_t seq_num[2];
static uint8_t self_led_state;

struct worker_data {
	uint8_t channel;
	uint8_t retries;
} __aligned(4);

K_MSGQ_DEFINE(kwork_msgq, sizeof(struct worker_data), CONFIG_BT_ISO_MAX_CHAN, 4);

int ble_acl_gateway_conn_peer_get(uint8_t chan_number, struct bt_conn **p_conn)
{
	if (chan_number >= CONFIG_BT_MAX_CONN) {
		return -EINVAL;
	}
	*p_conn = gateway_conn_peer[chan_number];
	return 0;
}

int ble_acl_gateway_conn_peer_set(uint8_t chan_number, struct bt_conn **p_conn)
{
	if (chan_number >= CONFIG_BT_MAX_CONN) {
		return -EINVAL;
	}

	if (gateway_conn_peer[chan_number] != NULL) {
		if (*p_conn == NULL) {
			gateway_conn_peer[chan_number] = NULL;
		} else {
			LOG_WRN("Already have a connection for peer: %d", chan_number);
		}
		/* Ignore duplicates as several peripherals might be
		 * advertising at the same time
		 */
		return 0;
	}

	gateway_conn_peer[chan_number] = *p_conn;
	return 0;
}

bool ble_acl_gateway_all_links_connected(void)
{
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (gateway_conn_peer[i] == NULL) {
			return false;
		}
	}
	return true;
}

static int hfclock_config_and_start(void)
{
	int ret;

	/* Use this to turn on 128 MHz clock for cpu_app */
	ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

	ret -= NRFX_ERROR_BASE_NUM;
	if (ret) {
		return ret;
	}

	nrfx_clock_hfclk_start();
	while (!nrfx_clock_hfclk_is_running()) {
	}

	return 0;
}

static void work_iso_cis_conn(struct k_work *work)
{
	int ret;
	struct bt_iso_connect_param connect_param;
	struct worker_data work_data;

	ret = k_msgq_get(&kwork_msgq, &work_data, K_NO_WAIT);
	if (ret) {
		LOG_ERR("k_msgq_get failed");
	}

	ret = ble_acl_gateway_conn_peer_get(work_data.channel, &connect_param.acl);
	if (ret) {
		LOG_ERR("Connection peer get error");
	}
	connect_param.iso_chan = iso_chan_p[work_data.channel];

	ret = bt_iso_chan_connect(&connect_param, 1);
	work_data.retries++;
	if (ret) {
		if (work_data.retries < CIS_CONN_RETRY_TIMES) {
			LOG_WRN("Got connect error from ch %d Retrying. code: %d count: %d",
				work_data.channel, ret, work_data.retries);
			ret = k_msgq_put(&kwork_msgq, &work_data, K_NO_WAIT);
			if (ret) {
				LOG_ERR("k_msgq_put failed");
			}
			/* Delay added to prevent controller overloading */
			k_work_reschedule(&iso_cis_conn_work, K_MSEC(500));
		} else {
			LOG_ERR("Could not connect ch %d after %d retries", work_data.channel,
				work_data.retries);
			bt_conn_disconnect(connect_param.acl, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
	}
}
static uint8_t iso_chan_to_idx(struct bt_iso_chan *chan)
{
	if (chan == NULL) {
		LOG_ERR("chan is NULL");
	}

	for (uint8_t i = 0; i < CONFIG_BT_ISO_MAX_CHAN; i++) {
		if (chan == iso_chan_p[i]) {
			return i;
		}
	}

	LOG_ERR("No index found for this channel");
	CODE_UNREACHABLE;
	return UINT8_MAX;
}

static void iso_send_work_handler(struct k_work *work)
{
	int ret;
	static uint8_t buf_data[ISO_MAX_PAYLOAD_TX];
	static bool hci_wrn_printed[2];
	struct net_buf *buf;
	struct bt_iso_tx_info tx_info = { 0 };
	static int prev_tx_info[2];
	static uint8_t led_toggling_state = 0;
	uint32_t last_sent;

	buf_data[0] = led_toggling_state;

	if (iso_chan[0].state == BT_ISO_STATE_CONNECTED) {
		/* Try to get the time when last ISO packet is sent */
		ret = bt_iso_chan_get_tx_sync(&iso_chan[0], &tx_info);
		if (tx_info.ts != 0 && !ret) {
			uint32_t curr_time = nrfx_timer_capture(
				&iso_sync_timer_instance, ISO_SYNC_TIMER_CURR_TIME_CAPTURE_CHANNEL);
			self_led_state = led_toggling_state;
			nrfx_timer_compare(
				&iso_sync_timer_instance, ISO_SYNC_TIMER_TIMER_TRIGGER_CHANNEL,
				(uint32_t)(curr_time + PRESENTATION_DELAY_US + SELF_OFFSET_US),
				true);
			/*
				ts0: the time when last ISO packet sent from CIS0
				diff: the diff time between two sent ISO pacekts
				last_sent: the diff between current time and last sent ISO packet
			*/
			last_sent = curr_time - tx_info.ts;
			LOG_WRN("ts0 %d, diff = %d, last_sent %d", tx_info.ts,
				tx_info.ts - prev_tx_info[0], last_sent);
			prev_tx_info[0] = tx_info.ts;
			if (last_sent < 2500 || last_sent > 7000) {
				LOG_ERR("Too close to the last begin or end of last sent packet, might not able to send ISO packets in the same interval");
			}
		}

		if (atomic_get(&iso_tx_pool_alloc[0]) >= HCI_ISO_BUF_ALLOC_PER_CHAN) {
			if (hci_wrn_printed[0]) {
				LOG_WRN("HCI ISO TX overrun on L ch - Single print");
				hci_wrn_printed[0] = true;
			}

			return;
		}
		hci_wrn_printed[0] = false;
		buf = net_buf_alloc(&tx_pool_l, K_FOREVER);

		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

		net_buf_add_mem(buf, buf_data, sizeof(buf_data));

		atomic_inc(&iso_tx_pool_alloc[0]);
		ret = bt_iso_chan_send(&iso_chan[0], buf, seq_num[0]++, BT_ISO_TIMESTAMP_NONE);

		if (ret < 0) {
			printk("Failed to send ISO data (%d)", ret);
			net_buf_unref(buf);
			atomic_dec(&iso_tx_pool_alloc[0]);
		} else {
			LOG_INF("Send dummy data to channel 0");
		}
	}

	if (iso_chan[1].state == BT_ISO_STATE_CONNECTED) {
		ret = bt_iso_chan_get_tx_sync(&iso_chan[1], &tx_info);
		if (tx_info.ts != 0 && !ret) {
			uint32_t curr_time = nrfx_timer_capture(
				&iso_sync_timer_instance, ISO_SYNC_TIMER_CURR_TIME_CAPTURE_CHANNEL);
			last_sent = curr_time - tx_info.ts;
			LOG_WRN("ts1 %d, diff = %d last_sent %d", tx_info.ts,
				tx_info.ts - prev_tx_info[1], last_sent);
			prev_tx_info[1] = tx_info.ts;
		}

		if (atomic_get(&iso_tx_pool_alloc[1]) >= HCI_ISO_BUF_ALLOC_PER_CHAN) {
			if (hci_wrn_printed[1]) {
				LOG_WRN("HCI ISO TX overrun on L ch - Single print");
				hci_wrn_printed[1] = true;
			}

			return;
		}
		hci_wrn_printed[1] = false;

		buf = net_buf_alloc(&tx_pool_r, K_FOREVER);
		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

		net_buf_add_mem(buf, buf_data, sizeof(buf_data));
		atomic_inc(&iso_tx_pool_alloc[1]);
		ret = bt_iso_chan_send(&iso_chan[1], buf, seq_num[1]++, BT_ISO_TIMESTAMP_NONE);

		if (ret < 0) {
			printk("Failed to send ISO data (%d)", ret);
			net_buf_unref(buf);
			atomic_dec(&iso_tx_pool_alloc[1]);
		} else {
			LOG_INF("Send dummy data to channel 1");
		}
	}
	led_toggling_state = !led_toggling_state;
	nrfx_timer_compare_int_enable(&led_toggling_timer_instance, NRF_TIMER_CC_CHANNEL0);
}

static int device_found(uint8_t type, const uint8_t *data, uint8_t data_len,
			const bt_addr_le_t *addr)
{
	int ret;
	struct bt_conn *conn;

	if ((data_len == DEVICE_NAME_PEER_L_LEN) &&
	    (strncmp(DEVICE_NAME_PEER_L, data, DEVICE_NAME_PEER_L_LEN) == 0)) {
		ret = bt_le_scan_stop();
		if (ret) {
			LOG_INF("Stop scan failed: %d", ret);
		}

		ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_MULTI,
					&conn);
		if (ret) {
			LOG_INF("Could not init connection");
			return ret;
		}
		ret = ble_acl_gateway_conn_peer_set(0, &conn);
		if (ret) {
			LOG_ERR("Connection peer set error %d", ret);
		}
		return 0;
	} else if ((data_len == DEVICE_NAME_PEER_R_LEN) &&
		   (strncmp(DEVICE_NAME_PEER_R, data, DEVICE_NAME_PEER_R_LEN) == 0)) {
		ret = bt_le_scan_stop();
		if (ret) {
			LOG_ERR("Stop scan failed: %d", ret);
		}

		ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_MULTI,
					&conn);
		if (ret) {
			LOG_INF("Could not init connection");
			return ret;
		}
		ret = ble_acl_gateway_conn_peer_set(1, &conn);
		if (ret) {
			LOG_ERR("Connection peer set error %d", ret);
		}
		return 0;
	}

	return -ENOENT;
}

/** @brief  Parse BLE advertisement package.
 */
static void ad_parse(struct net_buf_simple *p_ad, const bt_addr_le_t *addr)
{
	while (p_ad->len > 1) {
		uint8_t len = net_buf_simple_pull_u8(p_ad);
		uint8_t type;

		/* Check for early termination */
		if (len == 0) {
			return;
		}

		if (len > p_ad->len) {
			LOG_ERR("AD malformed");
			return;
		}

		type = net_buf_simple_pull_u8(p_ad);

		if (device_found(type, p_ad->data, len - 1, addr) == 0) {
			return;
		}

		(void)net_buf_simple_pull(p_ad, len - 1);
	}
}

static void on_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			    struct net_buf_simple *p_ad)
{
	/* Direct advertising has no payload, so no need to parse */
	if (type == BT_GAP_ADV_TYPE_ADV_IND || type == BT_GAP_ADV_TYPE_SCAN_RSP ||
	    type == BT_GAP_ADV_TYPE_EXT_ADV) {
		/* Note: May lead to connection creation */
		ad_parse(p_ad, addr);
	}
}

static void start_scan(void)
{
	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE_INTENSIVE, on_device_found);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return;
	}

	LOG_INF("Scanning successfully started");
}

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		     struct net_buf *buf)
{
	static int pkt_counter[2];
	if (chan == iso_chan_p[0]) {
		pkt_counter[0]++;
		if (pkt_counter[0] % 1000 == 0) {
			LOG_INF("Incoming data channel %p len %u, pkt received %d", chan, buf->len,
				pkt_counter[0]);
		}
	} else if (chan == iso_chan_p[1]) {
		pkt_counter[1]++;
		if (pkt_counter[1] % 1000 == 0) {
			LOG_INF("Incoming data channel %p len %u, pkt received %d", chan, buf->len,
				pkt_counter[1]);
		}
	}
}

static void iso_connected(struct bt_iso_chan *chan)
{
	LOG_INF("ISO Channel %p connected", chan);
	uint8_t chan_num;
	chan_num = iso_chan_to_idx(chan);

	LOG_INF("chan %d connected", chan_num);
	;
	nrfx_timer_compare_int_enable(&led_toggling_timer_instance, NRF_TIMER_CC_CHANNEL0);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	LOG_INF("ISO Channel %p disconnected (reason 0x%02x)", chan, reason);

	if (chan == iso_chan_p[0]) {
		atomic_clear(&iso_tx_pool_alloc[0]);
	} else if (chan == iso_chan_p[1]) {
		atomic_clear(&iso_tx_pool_alloc[1]);
	}
	nrfx_timer_compare_int_disable(&led_toggling_timer_instance, NRF_TIMER_CC_CHANNEL0);
}

static void iso_sent(struct bt_iso_chan *chan)
{
	if (chan == iso_chan_p[0]) {
		atomic_dec(&iso_tx_pool_alloc[0]);
	} else if (chan == iso_chan_p[1]) {
		atomic_dec(&iso_tx_pool_alloc[1]);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.recv = iso_recv,
	.connected = iso_connected,
	.disconnected = iso_disconnected,
	.sent = iso_sent,
};

int ble_trans_iso_cis_connect(struct bt_conn *conn)
{
	int ret;
	struct bt_conn *conn_active;

	for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		ret = ble_acl_gateway_conn_peer_get(i, &conn_active);
		if (ret) {
			LOG_ERR("Connection peer get error");
		}
		if (conn == conn_active) {
			struct worker_data work_data;

			work_data.channel = i;
			work_data.retries = 0;

			ret = k_msgq_put(&kwork_msgq, &work_data, K_NO_WAIT);
			if (ret) {
				return ret;
			}

			k_work_schedule(&iso_cis_conn_work, K_MSEC(500 * i));
		}
	}

	return 0;
}

static struct bt_iso_chan_io_qos iso_tx = {
	.sdu = ISO_MAX_PAYLOAD_TX,
	.phy = BT_GAP_LE_PHY_2M,
	.rtn = ISO_RTN_TIMES,
	.path = NULL,
};

static struct bt_iso_chan_io_qos iso_rx = {
	.sdu = ISO_MAX_PAYLOAD_RX,
	.phy = BT_GAP_LE_PHY_2M,
	.rtn = ISO_RTN_TIMES,
	.path = NULL,
};

static struct bt_iso_chan_qos iso_cis_qos = {
	.tx = &iso_tx,
	.rx = &iso_rx,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int ret;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Failed to connect to %s (%u)", addr, err);
	}
	LOG_INF("Connected: %s", addr);

	ret = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (ret) {
		LOG_ERR("set security failed %d", ret);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	struct bt_conn *conn_active;

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		int ret;

		ret = ble_acl_gateway_conn_peer_get(i, &conn_active);
		if (ret) {
			LOG_ERR("Connection peer get error");
		}
		if (conn_active == conn) {
			bt_conn_unref(conn_active);
			conn_active = NULL;
			ret = ble_acl_gateway_conn_peer_set(i, &conn_active);
			if (ret) {
				LOG_ERR("Connection peer get error %d", ret);
			}
			LOG_DBG("Headset %d disconnected", i);
			break;
		}
	}

	start_scan();
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	int ret;

	if (err) {
		LOG_ERR("MTU exchange failed, err = %d", err);
		ret = bt_conn_disconnect(conn, BT_HCI_ERR_LOCALHOST_TERM_CONN);
		if (ret) {
			LOG_ERR("Failed to disconnected, %d", ret);
		}
	} else {
		LOG_DBG("MTU exchange success");
		if (!ble_acl_gateway_all_links_connected()) {
			start_scan();
		} else {
			LOG_INF("All ACL links are connected");
			bt_le_scan_stop();
		}

		ret = ble_trans_iso_cis_connect(conn);
		if (ret) {
			LOG_ERR("Failed to connect to ISO CIS channel");
		}
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed_cb,
};

static struct bt_iso_cig_param cis_create_param = {
	.cis_channels = iso_chan_p,
	.num_cis = 2,
	.sca = BT_GAP_SCA_UNKNOWN,
	.packing = BT_ISO_PACKING_SEQUENTIAL,
	.framing = BT_ISO_FRAMING_UNFRAMED,
	.latency = 5, //ms
	.interval = ISO_INTERVAL_US,
};

int ble_trans_iso_cig_create(void)
{
	int ret;

	struct bt_iso_cig *cig;

	ret = bt_iso_cig_create(&cis_create_param, &cig);
	if (ret) {
		LOG_ERR("Failed to create CIG (%d)\n", ret);
		return ret;
	}

	return 0;
}
static void led_toggling_timer_event_handler(nrf_timer_event_t event_type, void *ctx)
{
	nrfx_timer_compare_int_disable(&led_toggling_timer_instance, NRF_TIMER_CC_CHANNEL0);
	k_work_submit(&iso_send_work);
}

static int led_toggling_timer_init()
{
	nrfx_err_t ret;

	ret = nrfx_timer_init(&led_toggling_timer_instance, &cfg, led_toggling_timer_event_handler);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx timer init error - Return value: %d", ret);
		return ret;
	}
	IRQ_CONNECT(TIMER2_IRQn, 5, nrfx_timer_2_irq_handler, NULL, 0);
	nrfx_timer_extended_compare(&led_toggling_timer_instance, NRF_TIMER_CC_CHANNEL0,
				    LED_TOGGLE_INTERVAL_US, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
				    false);
	nrfx_timer_enable(&led_toggling_timer_instance);

	return 0;
}

static void iso_sync_timer_event_handler(nrf_timer_event_t event_type, void *ctx)
{
	if (event_type == NRF_TIMER_EVENT_COMPARE0) {
		dk_set_led(DK_LED2, self_led_state);
	}
}

static int iso_sync_timer_init(const struct device *unused)
{
	nrfx_err_t ret;

	ARG_UNUSED(unused);

	ret = nrfx_timer_init(&iso_sync_timer_instance, &cfg, iso_sync_timer_event_handler);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx timer init error - Return value: %d", ret);
		return ret;
	}
	IRQ_CONNECT(TIMER1_IRQn, 4, nrfx_timer_1_irq_handler, NULL, 0);
	nrfx_timer_enable(&iso_sync_timer_instance);

	/* Initialize functionality for synchronization between APP and NET core */
	ret = nrfx_dppi_channel_alloc(&dppi_channel_timer_clear);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel alloc error (timer clear) - Return value: %d", ret);
		return ret;
	}

	nrf_ipc_publish_set(NRF_IPC, ISO_SYNC_TIMER_NET_APP_IPC_EVT, dppi_channel_timer_clear);
	nrf_timer_subscribe_set(iso_sync_timer_instance.p_reg, NRF_TIMER_TASK_CLEAR,
				dppi_channel_timer_clear);
	ret = nrfx_dppi_channel_enable(dppi_channel_timer_clear);
	if (ret - NRFX_ERROR_BASE_NUM) {
		LOG_ERR("nrfx DPPI channel enable error (timer clear) - Return value: %d", ret);
		return ret;
	}

	LOG_DBG("ISO sync timer initialized");

	return 0;
}

void main(void)
{
	int err;

	hfclock_config_and_start();

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
	err = dk_leds_init();
	if (err) {
		printk("LEDs init failed (err %d)\n", err);
		return;
	}
	LOG_INF("Bluetooth initialized");

	for (int8_t i = 0; i < CONFIG_BT_ISO_MAX_CHAN; i++) {
		iso_chan_p[i] = &iso_chan[i];
	}

	for (int i = 0; i < CONFIG_BT_ISO_MAX_CHAN; i++) {
		iso_chan_p[i]->ops = &iso_ops;
		iso_chan_p[i]->qos = &iso_cis_qos;
	}

	err = ble_trans_iso_cig_create();
	if (err) {
		LOG_ERR("CIG create failed %d", err);
	}

	k_work_init_delayable(&iso_cis_conn_work, work_iso_cis_conn);
	k_work_init(&iso_send_work, iso_send_work_handler);
	led_toggling_timer_init();
	start_scan();
}

SYS_INIT(iso_sync_timer_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
