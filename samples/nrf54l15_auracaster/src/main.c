/*
 * Copyright (c) 2022-2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/autoconf.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys_clock.h>
#include <zephyr/toolchain.h>
#include "sw_codec_lc3.h"
#include <math.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <hal/nrf_clock.h>

BUILD_ASSERT(strlen(CONFIG_BROADCAST_CODE) <= BT_ISO_BROADCAST_CODE_SIZE, "Invalid broadcast code");

/* Zephyr Controller works best while Extended Advertising interval to be a multiple
 * of the ISO Interval minus 10 ms (max. advertising random delay). This is
 * required to place the AUX_ADV_IND PDUs in a non-overlapping interval with the
 * Broadcast ISO radio events.
 *
 * I.e. for a 7.5 ms ISO interval use 90 ms minus 10 ms ==> 80 ms advertising
 * interval.
 * And, for 10 ms ISO interval, can use 90 ms minus 10 ms ==> 80 ms advertising
 * interval.
 */
#define BT_LE_EXT_ADV_CUSTOM                                                                       \
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV, BT_GAP_MS_TO_ADV_INTERVAL(80),                      \
			BT_GAP_MS_TO_ADV_INTERVAL(80), NULL)

/* When BROADCAST_ENQUEUE_COUNT > 1 we can enqueue enough buffers to ensure that
 * the controller is never idle
 */
#define BROADCAST_ENQUEUE_COUNT 3U
#define TOTAL_BUF_NEEDED	(BROADCAST_ENQUEUE_COUNT * CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT)

BUILD_ASSERT(CONFIG_BT_ISO_TX_BUF_COUNT >= TOTAL_BUF_NEEDED,
	     "CONFIG_BT_ISO_TX_BUF_COUNT should be at least "
	     "BROADCAST_ENQUEUE_COUNT * CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT");

static struct bt_bap_lc3_preset preset_active = BT_BAP_LC3_BROADCAST_PRESET_16_2_1(
	BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT,
	BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

#define BROADCAST_SAMPLE_RATE 16000
#define MAX_SAMPLE_RATE	      16000

#define MAX_FRAME_DURATION_US 10000
#define MAX_NUM_SAMPLES	      ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)

#define AUDIO_VOLUME		(INT16_MAX - 3000) /* codec does clipping above INT16_MAX - 3000 */
#define AUDIO_TONE_FREQUENCY_HZ 400

#define MAX_SAMPLE_RATE	 16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT	 1000
/* Size of a block for 10 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels)                                              \
	(BYTES_PER_SAMPLE * (_sample_rate / 100) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE BLOCK_SIZE(MAX_SAMPLE_RATE, 2)
#define BLOCK_COUNT    8
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 8);
static const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));

#define LC3_ENCODER_STACK_SIZE (4 * 4096)
#define LC3_ENCODER_PRIORITY   5

static void init_lc3_thread(void *arg1, void *arg2, void *arg3);
K_THREAD_DEFINE(encoder, LC3_ENCODER_STACK_SIZE, init_lc3_thread, NULL, NULL, NULL,
		LC3_ENCODER_PRIORITY, 0, -1);

static struct broadcast_source_stream {
	struct bt_bap_stream stream;
	uint16_t seq_num;
	size_t sent_cnt;
} streams[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
static struct bt_bap_broadcast_source *broadcast_source;

NET_BUF_POOL_FIXED_DEFINE(tx_pool, TOTAL_BUF_NEEDED, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static int16_t send_pcm_data[MAX_NUM_SAMPLES];

static bool stopping;

static K_SEM_DEFINE(sem_started, 0U, 1U);
static K_SEM_DEFINE(sem_stopped, 0U, 1U);

static int freq_hz;
static int frame_duration_us;
static int frames_per_sdu;
static int octets_per_frame;

static K_SEM_DEFINE(lc3_encoder_sem, 0U, TOTAL_BUF_NEEDED);

static uint16_t pcm_bytes_req_enc;
static void send_data(struct broadcast_source_stream *source_stream)
{
	struct bt_bap_stream *stream = &source_stream->stream;
	struct net_buf *buf;
	int ret;

	if (stopping) {
		return;
	}

	buf = net_buf_alloc(&tx_pool, K_FOREVER);
	if (buf == NULL) {
		printk("Could not allocate buffer when sending on %p\n", stream);
		return;
	}

	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

	uint8_t lc3_encoded_buffer[preset_active.qos.sdu];
	uint16_t encoded_bytes_written;

	ret = sw_codec_lc3_enc_run(send_pcm_data, sizeof(send_pcm_data), octets_per_frame * 8 * 100,
				   0, sizeof(lc3_encoded_buffer), lc3_encoded_buffer,
				   &encoded_bytes_written);
	if (ret) {
		printk("LC3 encoder failed - wrong parameters?: %d", ret);
		net_buf_unref(buf);
		return;
	}

	net_buf_add_mem(buf, lc3_encoded_buffer, preset_active.qos.sdu);

	ret = bt_bap_stream_send(stream, buf, source_stream->seq_num++);
	if (ret < 0) {
		/* This will end broadcasting on this stream. */
		printk("Unable to broadcast data on %p: %d\n", stream, ret);
		net_buf_unref(buf);
		return;
	}

	source_stream->sent_cnt++;
	if ((source_stream->sent_cnt % 1000U) == 0U) {
		printk("Stream %p: Sent %u total ISO packets\n", stream, source_stream->sent_cnt);
	}
}

static void init_lc3_thread(void *arg1, void *arg2, void *arg3)
{
	int ret;
	const struct bt_audio_codec_cfg *codec_cfg = &preset_active.codec_cfg;
	static void *buffer;

	printk("init_lc3_thread\n");

	ret = bt_audio_codec_cfg_get_freq(codec_cfg);
	if (ret > 0) {
		freq_hz = bt_audio_codec_cfg_freq_to_freq_hz(ret);
	} else {
		return;
	}

	ret = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
	if (ret > 0) {
		frame_duration_us = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);
	} else {
		printk("Error: Frame duration not set, cannot start codec.");
		return;
	}

	octets_per_frame = bt_audio_codec_cfg_get_octets_per_frame(codec_cfg);
	frames_per_sdu = bt_audio_codec_cfg_get_frame_blocks_per_sdu(codec_cfg, true);

	if (freq_hz < 0) {
		printk("Error: Codec frequency not set, cannot start codec.");
		return;
	}

	if (frame_duration_us < 0) {
		printk("Error: Frame duration not set, cannot start codec.");
		return;
	}

	if (octets_per_frame < 0) {
		printk("Error: Octets per frame not set, cannot start codec.");
		return;
	}

	ret = sw_codec_lc3_init(NULL, NULL, MAX_FRAME_DURATION_US);
	if (ret) {
		printk("sw_codec_lc3_init failed (ret %d)\n", ret);
	}
	ret = sw_codec_lc3_enc_init(freq_hz, 16, MAX_FRAME_DURATION_US, octets_per_frame * 8 * 100,
				    ARRAY_SIZE(streams), &pcm_bytes_req_enc);
	if (ret) {
		printk("sw_codec_lc3_enc_init failed (ret %d)\n", ret);
	} else {
		printk("LC3 encoder initialized, PCM bytes required for encoding: %u\n",
		       pcm_bytes_req_enc);
	}

	uint32_t size;

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		printk("START trigger failed: %d\n", ret);
	}

	while (true) {
		for (size_t i = 0U; i < ARRAY_SIZE(streams); i++) {
			k_sem_take(&lc3_encoder_sem, K_FOREVER);
		}

		ret = dmic_read(dmic_dev, 0, &buffer, &size, 100);
		if (ret < 0) {
			printk("read failed: %d\n", ret);
		}
		memcpy(send_pcm_data, buffer, sizeof(send_pcm_data));

		k_mem_slab_free(&mem_slab, buffer);

		for (size_t i = 0U; i < ARRAY_SIZE(streams); i++) {
			send_data(&streams[i]);
		}
	}
}

static void stream_started_cb(struct bt_bap_stream *stream)
{
	struct broadcast_source_stream *source_stream =
		CONTAINER_OF(stream, struct broadcast_source_stream, stream);

	source_stream->seq_num = 0U;
	source_stream->sent_cnt = 0U;
}

static void stream_sent_cb(struct bt_bap_stream *stream)
{
	k_sem_give(&lc3_encoder_sem);
}

static struct bt_bap_stream_ops stream_ops = {
	.started = stream_started_cb,
	.sent = stream_sent_cb,
};

static int setup_broadcast_source(struct bt_bap_broadcast_source **source)
{
	struct bt_bap_broadcast_source_stream_param
		stream_params[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
	struct bt_bap_broadcast_source_subgroup_param
		subgroup_param[CONFIG_BT_BAP_BROADCAST_SRC_SUBGROUP_COUNT];
	struct bt_bap_broadcast_source_param create_param = {0};
	const size_t streams_per_subgroup = ARRAY_SIZE(stream_params) / ARRAY_SIZE(subgroup_param);
	uint8_t left[] = {BT_AUDIO_CODEC_DATA(BT_AUDIO_CODEC_CFG_CHAN_ALLOC,
					      BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_LEFT))};
	uint8_t right[] = {BT_AUDIO_CODEC_DATA(BT_AUDIO_CODEC_CFG_CHAN_ALLOC,
					       BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_RIGHT))};
	int err;

	for (size_t i = 0U; i < ARRAY_SIZE(subgroup_param); i++) {
		subgroup_param[i].params_count = streams_per_subgroup;
		subgroup_param[i].params = stream_params + i * streams_per_subgroup;
		subgroup_param[i].codec_cfg = &preset_active.codec_cfg;
	}

	for (size_t j = 0U; j < ARRAY_SIZE(stream_params); j++) {
		stream_params[j].stream = &streams[j].stream;
		stream_params[j].data = j == 0 ? left : right;
		stream_params[j].data_len = j == 0 ? sizeof(left) : sizeof(right);
		bt_bap_stream_cb_register(stream_params[j].stream, &stream_ops);
	}

	create_param.params_count = ARRAY_SIZE(subgroup_param);
	create_param.params = subgroup_param;
	create_param.qos = &preset_active.qos;
	create_param.encryption = strlen(CONFIG_BROADCAST_CODE) > 0;
	create_param.packing = BT_ISO_PACKING_SEQUENTIAL;

	if (create_param.encryption) {
		memcpy(create_param.broadcast_code, CONFIG_BROADCAST_CODE,
		       strlen(CONFIG_BROADCAST_CODE));
	}

	printk("Creating broadcast source with %zu subgroups with %zu streams\n",
	       ARRAY_SIZE(subgroup_param), ARRAY_SIZE(subgroup_param) * streams_per_subgroup);

	err = bt_bap_broadcast_source_create(&create_param, source);
	if (err != 0) {
		printk("Unable to create broadcast source: %d\n", err);
		return err;
	}

	return 0;
}

static void source_started_cb(struct bt_bap_broadcast_source *source)
{
	printk("Broadcast source %p started\n", source);
	k_sem_give(&sem_started);
}

static void source_stopped_cb(struct bt_bap_broadcast_source *source, uint8_t reason)
{
	printk("Broadcast source %p stopped with reason 0x%02X\n", source, reason);
	k_sem_give(&sem_stopped);
}

int clocks_start(void)
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

int main(void)
{
	int err;
	static struct bt_bap_broadcast_source_cb broadcast_source_cb = {
		.started = source_started_cb,
		.stopped = source_stopped_cb,
	};
	struct bt_le_ext_adv *adv;

	clocks_start();	// Disable this line would trigger the issue

	if (!device_is_ready(dmic_dev)) {
		printk("%s is not ready\n", dmic_dev->name);
		return 0;
	}

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io =
			{
				/* These fields can be used to limit the PDM clock
				 * configurations that the driver is allowed to use
				 * to those supported by the microphone.
				 */
				.min_pdm_clk_freq = 1000000,
				.max_pdm_clk_freq = 3500000,
				.min_pdm_clk_dc = 40,
				.max_pdm_clk_dc = 60,
			},
		.streams = &stream,
		.channel =
			{
				.req_num_streams = 1,
			},
	};

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
	cfg.streams[0].block_size = BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	err = dmic_configure(dmic_dev, &cfg);
	if (err < 0) {
		printk("Failed to configure the driver: %d\n", err);
		return err;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	printk("Bluetooth initialized\n");

	err = bt_bap_broadcast_source_register_cb(&broadcast_source_cb);
	if (err != 0) {
		printk("Failed to register broadcast source callbacks (err %d)\n", err);
		return 0;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(send_pcm_data); i++) {
		/* Initialize mock data */
		send_pcm_data[i] = i;
	}

	/* Broadcast Audio Streaming Endpoint advertising data */
	NET_BUF_SIMPLE_DEFINE(ad_buf, BT_UUID_SIZE_16 + BT_AUDIO_BROADCAST_ID_SIZE);
	NET_BUF_SIMPLE_DEFINE(base_buf, 128);
	struct bt_data ext_ad[2];
	struct bt_data per_ad;
	uint32_t broadcast_id;

	/* Create a connectable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_CUSTOM, NULL, &adv);
	if (err != 0) {
		printk("Unable to create extended advertising set: %d\n", err);
		return 0;
	}

	/* Set periodic advertising parameters */
	err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_DEFAULT);
	if (err) {
		printk("Failed to set periodic advertising parameters (err %d)\n", err);
		return 0;
	}

	printk("Creating broadcast source\n");
	err = setup_broadcast_source(&broadcast_source);
	if (err != 0) {
		printk("Unable to setup broadcast source: %d\n", err);
		return 0;
	}

#if defined(CONFIG_STATIC_BROADCAST_ID)
	broadcast_id = CONFIG_BROADCAST_ID;
#else
	err = bt_rand(&broadcast_id, BT_AUDIO_BROADCAST_ID_SIZE);
	if (err) {
		printk("Unable to generate broadcast ID: %d\n", err);
		return err;
	}
#endif /* CONFIG_STATIC_BROADCAST_ID */

	/* Setup extended advertising data */
	net_buf_simple_add_le16(&ad_buf, BT_UUID_BROADCAST_AUDIO_VAL);
	net_buf_simple_add_le24(&ad_buf, broadcast_id);
	ext_ad[0].type = BT_DATA_SVC_DATA16;
	ext_ad[0].data_len = ad_buf.len;
	ext_ad[0].data = ad_buf.data;
	ext_ad[1] = (struct bt_data)BT_DATA(BT_DATA_BROADCAST_NAME, CONFIG_BT_DEVICE_NAME,
					    sizeof(CONFIG_BT_DEVICE_NAME) - 1);
	err = bt_le_ext_adv_set_data(adv, ext_ad, 2, NULL, 0);
	if (err != 0) {
		printk("Failed to set extended advertising data: %d\n", err);
		return 0;
	}

	/* Setup periodic advertising data */
	err = bt_bap_broadcast_source_get_base(broadcast_source, &base_buf);
	if (err != 0) {
		printk("Failed to get encoded BASE: %d\n", err);
		return 0;
	}

	per_ad.type = BT_DATA_SVC_DATA16;
	per_ad.data_len = base_buf.len;
	per_ad.data = base_buf.data;
	err = bt_le_per_adv_set_data(adv, &per_ad, 1);
	if (err != 0) {
		printk("Failed to set periodic advertising data: %d\n", err);
		return 0;
	}

	/* Start extended advertising */
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising: %d\n", err);
		return 0;
	}

	/* Enable Periodic Advertising */
	err = bt_le_per_adv_start(adv);
	if (err) {
		printk("Failed to enable periodic advertising: %d\n", err);
		return 0;
	}

	printk("Starting broadcast source\n");
	stopping = false;
	err = bt_bap_broadcast_source_start(broadcast_source, adv);
	if (err != 0) {
		printk("Unable to start broadcast source: %d\n", err);
		return 0;
	}

	/* Wait for broadcast source to be started */
	k_sem_take(&sem_started, K_FOREVER);
	k_thread_start(encoder);
	printk("Broadcast source started\n");

	/* Initialize sending */
	for (size_t i = 0U; i < ARRAY_SIZE(streams); i++) {
		for (unsigned int j = 0U; j < BROADCAST_ENQUEUE_COUNT; j++) {
			stream_sent_cb(&streams[i].stream);
		}
	}
	while (true) {
		k_sleep(K_MSEC(1000));
	}
	return 0;
}
