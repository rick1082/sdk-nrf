/*
 * Copyright (c) 2021-2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/audio/dmic.h>
#include "hal/nrf_pdm.h"
#include "sw_codec_lc3.h"

#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */

/* Audio configuration */
#define MAX_SAMPLE_RATE			16000
#define MAX_FRAME_DURATION_US		10000
#define MAX_NUM_SAMPLES			((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)
#define TOTAL_BUF_NEEDED		2
#define SAMPLE_BIT_WIDTH		16
#define BYTES_PER_SAMPLE		sizeof(int16_t)
#define READ_TIMEOUT			1000

#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	(BYTES_PER_SAMPLE * (_sample_rate / 100) * _number_of_channels)

#define MAX_BLOCK_SIZE			BLOCK_SIZE(MAX_SAMPLE_RATE, 2)
#define BLOCK_COUNT			8

/* Global variables */
static struct bt_conn *peripheral_conn[2];
static struct k_work_delayable iso_send_work;
static struct bt_iso_chan iso_chan[2];
static uint16_t seq_num;
static uint16_t latency_ms = 10U;
static uint32_t interval_us = 10U * USEC_PER_MSEC;

static K_SEM_DEFINE(lc3_encoder_sem, 0U, TOTAL_BUF_NEEDED);
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 8);

NET_BUF_POOL_FIXED_DEFINE(tx_pool, 1, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
static const char srch_name[] = "Zephyr ISO server";
static int16_t send_pcm_data[MAX_NUM_SAMPLES];

#define LC3_ENCODER_STACK_SIZE 8192
#define LC3_ENCODER_PRIORITY   5
static void dmic_fetch_thread(void *arg1, void *arg2, void *arg3);
K_THREAD_DEFINE(dmic_fetch, LC3_ENCODER_STACK_SIZE, dmic_fetch_thread, NULL, NULL, NULL,
		LC3_ENCODER_PRIORITY, 0, -1);

static void iso_sending_thread(void *arg1, void *arg2, void *arg3);
K_THREAD_DEFINE(iso_sending, 4096, iso_sending_thread, NULL, NULL, NULL,
		7, 0, -1);
K_SEM_DEFINE(chan0_iso_sent_sem, 0U, 1U);
K_SEM_DEFINE(chan1_iso_sent_sem, 0U, 1U);
/* Function declarations */
static void start_scan(void);

/**
 * @brief Get connection index for a given connection
 */
static int conn_num_get(struct bt_conn *conn)
{
	for (size_t i = 0; i < ARRAY_SIZE(peripheral_conn); i++) {
		if (peripheral_conn[i] == conn) {
			return i;
		}
	}
	return -1;
}

/**
 * @brief Check if all connections are established
 */
static bool all_conn_connected(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(peripheral_conn); i++) {
		if (peripheral_conn[i] == NULL) {
			return false;
		}
	}
	return true;
}

/**
 * @brief Get first empty connection slot
 */
static int conn_empty_slot_get(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(peripheral_conn); i++) {
		if (peripheral_conn[i] == NULL) {
			return i;
		}
	}
	return -1;
}

/**
 * @brief Check if all ISO channels are connected
 */
static bool all_iso_connected(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(iso_chan); i++) {
		if (iso_chan[i].state != BT_ISO_STATE_CONNECTED) {
			return false;
		}
	}
	return true;
}

/**
 * @brief Check if all ISO channels are disconnected
 */
static bool all_iso_disconnected(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(iso_chan); i++) {
		if (iso_chan[i].state != BT_ISO_STATE_DISCONNECTED) {
			return false;
		}
	}
	return true;
}

/**
 * @brief ISO timer timeout handler
 */
static void iso_timer_timeout(struct k_work *work)
{
	//printk("ISO timer timeout\n");
	k_work_schedule(&iso_send_work, K_MSEC(1000));
}

/**
 * @brief Check device name in advertisement data
 */
static bool device_name_check(struct bt_data *data, void *user_data)
{
	int ret;
	bt_addr_le_t *addr = user_data;
	char addr_string[BT_ADDR_LE_STR_LEN];

	const struct bt_le_conn_param *conn_param =
		BT_LE_CONN_PARAM(BT_GAP_MS_TO_CONN_INTERVAL(60),
				 BT_GAP_MS_TO_CONN_INTERVAL(60), 0,
				 BT_GAP_MS_TO_CONN_TIMEOUT(4000));

	if (data->type == BT_DATA_NAME_COMPLETE || data->type == BT_DATA_NAME_SHORTENED) {
		size_t srch_name_size = strlen(srch_name);
		
		if (memcmp(srch_name, data->data, srch_name_size - 1) == 0) {
			ret = bt_le_scan_stop();
			if (ret) {
				printk("Stop scan failed: %d\n", ret);
			}

			bt_addr_le_to_str(addr, addr_string, BT_ADDR_LE_STR_LEN);
			printk("Creating connection to device: %s\n", addr_string);

			ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, conn_param,
						&peripheral_conn[conn_empty_slot_get()]);
			if (ret) {
				printk("Create conn to %s failed (%u)\n", addr_string, ret);
				start_scan();
			}
			return false;
		}
	}
	return true;
}

/**
 * @brief Handle discovered device
 */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	if (all_conn_connected()) {
		return;
	}

	if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	bt_data_parse(ad, device_name_check, (void *)addr);
}

static void iso_sending_thread(void *arg1, void *arg2, void *arg3)
{
	struct bt_iso_tx_info tx_info[2];
	uint8_t dummy_data[100] = {0};
	while(1){
		for(int i = 0; i < ARRAY_SIZE(iso_chan); i++){
			if (iso_chan[i].state == BT_ISO_STATE_CONNECTED) {
				k_sem_take(i == 0 ? &chan0_iso_sent_sem : &chan1_iso_sent_sem, K_MSEC(20));
				struct net_buf *buf = net_buf_alloc(&tx_pool, K_FOREVER);
				net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
				net_buf_add_mem(buf, dummy_data, sizeof(dummy_data));
				tx_info[i].seq_num = seq_num++;
				int err = bt_iso_chan_send(&iso_chan[i], buf, &tx_info[i]);
				if (err < 0) {
					printk("Failed to send ISO data: %d\n", err);
					net_buf_unref(buf);
				} else {
					//printk("Sent ISO data on chan %d\n", i);
				}
			}
		}
		k_sleep(K_MSEC(1));
	}
}

static void dmic_fetch_thread(void *arg1, void *arg2, void *arg3)
{
	int ret;
	void *buffer;
	uint32_t size;

	while (true) {
		//k_sem_take(&lc3_encoder_sem, K_FOREVER);

		ret = dmic_read(dmic_dev, 0, &buffer, &size, 10);
		if (ret < 0) {
			printk("DMIC read failed: %d\n", ret);
			k_mem_slab_free(&mem_slab, buffer);
			dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
			//dmic_trigger(dmic_dev, DMIC_TRIGGER_RESET);
			k_yield();
			k_sleep(K_MSEC(1000));
			dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
		}
		if (size > sizeof(send_pcm_data)) {
			printk("Buffer size exceeds send_pcm_data size\n");
			size = sizeof(send_pcm_data);
		}
		memcpy(send_pcm_data, buffer, size);
		//printk("DMIC read %d bytes\n", size);
		k_mem_slab_free(&mem_slab, buffer);
		//printk("DMIC buffer freed\n");
		//send_data();
		//k_sleep(K_MSEC(1));
	}
}

/**
 * @brief Initialize PDM microphone
 */
static int pdm_mic_init(void)
{
	int err;
	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3250000,
			.min_pdm_clk_dc = 40,
			.max_pdm_clk_dc = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};

	err = device_is_ready(dmic_dev);
	if (err < 0) {
		printk("DMIC device is not ready: %d\n", err);
		return err;
	}

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
	cfg.streams[0].block_size = BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	err = dmic_configure(dmic_dev, &cfg);
	if (err < 0) {
		printk("Failed to configure the driver: %d\n", err);
		return err;
	}

	return 0;
}

/**
 * @brief Start BLE scanning
 */
static void start_scan(void)
{
	int err;

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}


K_MUTEX_DEFINE(dmic_thread_mutex);
/**
 * @brief ISO channel connected callback
 */
static void iso_connected(struct bt_iso_chan *chan)
{
	const struct bt_iso_chan_path hci_path = {
		.pid = BT_ISO_DATA_PATH_HCI,
		.format = BT_HCI_CODING_FORMAT_TRANSPARENT,
	};
	int err;

	printk("ISO Channel %p connected\n", chan);

	seq_num = 0U;

	err = bt_iso_setup_data_path(chan, BT_HCI_DATAPATH_DIR_HOST_TO_CTLR, &hci_path);
	if (err != 0) {
		printk("Failed to setup ISO TX data path: %d\n", err);
	} else {
		k_mutex_lock(&dmic_thread_mutex, K_FOREVER);
		if (iso_chan[0].state != BT_ISO_STATE_CONNECTED ||
		    iso_chan[1].state != BT_ISO_STATE_CONNECTED) {
			printk("One ISO channel connected, starting ISO send timer\n");
			k_work_schedule(&iso_send_work, K_NO_WAIT);
			k_thread_start(dmic_fetch);
			k_thread_resume(dmic_fetch);
			err = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
			if (err < 0) {
				printk("DMIC start trigger failed: %d\n", err);
			} else {
				printk("DMIC start trigger success\n");
				//k_sem_give(&lc3_encoder_sem);
			}
		}
		k_mutex_unlock(&dmic_thread_mutex);
	}
}

/**
 * @brief ISO channel disconnected callback
 */
static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	int err;

	printk("ISO Channel %p disconnected (reason 0x%02x)\n", chan, reason);

	err = bt_iso_remove_data_path(chan, BT_HCI_DATAPATH_DIR_HOST_TO_CTLR);
	if (err != 0) {
		printk("Failed to setup ISO TX data path: %d\n", err);
	}

	if (all_iso_disconnected()) {
		printk("All ISO channels disconnected, stop iso_send_work\n");
		k_work_cancel_delayable(&iso_send_work);
		k_thread_suspend(dmic_fetch);
		err = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
		if (err < 0) {
			printk("DMIC stop trigger failed: %d\n", err);
		} else {
			printk("DMIC stop trigger success\n");
		}
	}
}


/**
 * @brief ISO data sent callback
 */
static void iso_sent(struct bt_iso_chan *chan)
{
	//printk("ISO Channel %p sent\n", chan);
	if (chan == &iso_chan[0]) {
		k_sem_give(&chan0_iso_sent_sem);
	} else if (chan == &iso_chan[1]) {
		k_sem_give(&chan1_iso_sent_sem);
	}
}

static struct bt_iso_chan_ops iso_ops = {
	.connected = iso_connected,
	.disconnected = iso_disconnected,
	.sent = iso_sent,
};

static struct bt_iso_chan_io_qos iso_tx = {
	.sdu = CONFIG_BT_ISO_TX_MTU,
	.phy = BT_GAP_LE_PHY_2M,
	.rtn = 1,
};

static struct bt_iso_chan_qos iso_qos = {
	.tx = &iso_tx,
	.rx = NULL,
};

struct iso_create_item {
	struct k_work iso_create_work;
	int index;
	struct bt_conn *conn;
};

/**
 * @brief Work handler for creating ISO connection
 */
void iso_create_work_handler(struct k_work *work)
{
	int iso_err;
	struct iso_create_item *item = CONTAINER_OF(work, struct iso_create_item, iso_create_work);
	
	printk("Creating ISO channel for conn %d\n", item->index);

	struct bt_iso_connect_param connect_param = {
		.acl = item->conn,
		.iso_chan = &iso_chan[item->index],
	};

	iso_err = bt_iso_chan_connect(&connect_param, 1);
	if (iso_err) {
		printk("Failed to connect iso (%d)\n", iso_err);
		k_sleep(K_MSEC(100));
		k_work_submit(&item->iso_create_work);
	}
}

/**
 * @brief Connection established callback
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	static struct iso_create_item item;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s %u %s\n", addr, err, bt_hci_err_to_str(err));
		bt_conn_unref(peripheral_conn[conn_num_get(conn)]);
		peripheral_conn[conn_num_get(conn)] = NULL;
		start_scan();
		return;
	}

	printk("Connected: %s\n", addr);

	item.index = conn_num_get(conn);
	item.conn = conn;
	k_work_init(&item.iso_create_work, iso_create_work_handler);
	k_work_submit(&item.iso_create_work);

	if (all_conn_connected()) {
		printk("All connections established\n");
	} else {
		start_scan();
	}
}

/**
 * @brief Connection lost callback
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Disconnected: %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

	bt_conn_unref(peripheral_conn[conn_num_get(conn)]);
	peripheral_conn[conn_num_get(conn)] = NULL;

	start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		printk("Unable to get the Clock manager\n");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		printk("Clock request failed: %d\n", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			printk("Clock could not be started: %d\n", res);
			return res;
		}
	} while (err);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	printk("HF clock started\n");
	return 0;
}

/**
 * @brief Main application entry point
 */
int main(void)
{
	int err;
	struct bt_iso_chan *channels[2];
	struct bt_iso_cig_param param;
	struct bt_iso_cig *cig;

	clocks_start();

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	pdm_mic_init();

	printk("Bluetooth initialized\n");

	/* Initialize ISO channels */
	iso_chan[0].ops = &iso_ops;
	iso_chan[0].qos = &iso_qos;
	iso_chan[1].ops = &iso_ops;
	iso_chan[1].qos = &iso_qos;

#if defined(CONFIG_BT_SMP)
	iso_chan[0].required_sec_level = BT_SECURITY_L2;
	iso_chan[1].required_sec_level = BT_SECURITY_L2;
#endif /* CONFIG_BT_SMP */

	/* Create CIG */
	channels[0] = &iso_chan[0];
	channels[1] = &iso_chan[1];
	param.cis_channels = channels;
	param.num_cis = ARRAY_SIZE(channels);
	param.sca = BT_GAP_SCA_UNKNOWN;
	param.packing = 0;
	param.framing = 0;
	param.c_to_p_latency = latency_ms;
	param.p_to_c_latency = latency_ms;
	param.c_to_p_interval = interval_us;
	param.p_to_c_interval = interval_us;

	err = bt_iso_cig_create(&param, &cig);
	if (err != 0) {
		printk("Failed to create CIG (%d)\n", err);
		return 0;
	}

	k_work_init_delayable(&iso_send_work, iso_timer_timeout);
	
	start_scan();
	k_thread_start(iso_sending);
	
	return 0;
}
