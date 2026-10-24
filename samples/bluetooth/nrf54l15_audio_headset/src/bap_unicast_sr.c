/** @file
 *  @brief Bluetooth Basic Audio Profile (BAP) Unicast Server role.
 *
 *  Copyright (c) 2021-2026 Nordic Semiconductor ASA
 *  Copyright (c) 2022 Codecoup
 *  Copyright (c) 2023 NXP
 *
 *  SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/autoconf.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

#include "nrf54l15_audio_headset.h"

#define TX_LOG_INTERVAL 100U

static struct unicast_stream {
	struct bt_bap_stream stream;
	struct iso_rx_stats rx_stats;
	uint16_t tx_seq_num;
	size_t tx_cnt;
} sink_streams[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT], source_streams[CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT];

static const struct bt_bap_qos_cfg_pref qos_pref =
	BT_BAP_QOS_CFG_PREF(true, BT_GAP_LE_PHY_2M, 0x02, 10, 20000, 40000, 20000, 40000);

static void print_hex(const uint8_t *ptr, size_t len)
{
	while (len-- != 0) {
		printk("%02x", *ptr++);
	}
}

static bool print_cb(struct bt_data *data, void *user_data)
{
	const char *str = (const char *)user_data;

	printk("%s: type 0x%02x value_len %u: ", str, data->type, data->data_len);
	print_hex(data->data, data->data_len);
	printk("\n");

	return true;
}

static void print_codec_cfg(const struct bt_audio_codec_cfg *codec_cfg)
{
	printk("codec_cfg 0x%02x cid 0x%04x vid 0x%04x count %u\n", codec_cfg->id, codec_cfg->cid,
	       codec_cfg->vid, codec_cfg->data_len);

	if (codec_cfg->id == BT_HCI_CODING_FORMAT_LC3) {
		enum bt_audio_location chan_allocation;
		int ret;

		ret = bt_audio_codec_cfg_get_freq(codec_cfg);
		if (ret > 0) {
			printk("  Frequency: %d Hz\n", bt_audio_codec_cfg_freq_to_freq_hz(ret));
		}

		ret = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
		if (ret > 0) {
			printk("  Frame Duration: %d us\n",
			       bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret));
		}

		ret = bt_audio_codec_cfg_get_chan_allocation(codec_cfg, &chan_allocation, false);
		if (ret == 0) {
			printk("  Channel allocation: 0x%x\n", chan_allocation);
		}

		printk("  Octets per frame: %d (negative means value not present)\n",
		       bt_audio_codec_cfg_get_octets_per_frame(codec_cfg));
		printk("  Frames per SDU: %d\n",
		       bt_audio_codec_cfg_get_frame_blocks_per_sdu(codec_cfg, true));
	} else {
		print_hex(codec_cfg->data, codec_cfg->data_len);
		printk("\n");
	}

	bt_audio_data_parse(codec_cfg->meta, codec_cfg->meta_len, print_cb, "meta");
}

static void print_qos(const struct bt_bap_qos_cfg *qos)
{
	printk("QoS: interval %u framing 0x%02x phy 0x%02x sdu %u "
	       "rtn %u latency %u pd %u\n",
	       qos->interval, qos->framing, qos->phy, qos->sdu,
	       qos->rtn, qos->latency, qos->pd);
}

static const char *dir_str(enum bt_audio_dir dir)
{
	return dir == BT_AUDIO_DIR_SINK ? "sink" : "source";
}

static struct bt_bap_stream *stream_alloc(enum bt_audio_dir dir)
{
	if (dir == BT_AUDIO_DIR_SINK) {
		ARRAY_FOR_EACH_PTR(sink_streams, unicast_stream) {
			if (unicast_stream->stream.ep == NULL) {
				return &unicast_stream->stream;
			}
		}
	} else if (dir == BT_AUDIO_DIR_SOURCE) {
		ARRAY_FOR_EACH_PTR(source_streams, unicast_stream) {
			if (unicast_stream->stream.ep == NULL) {
				return &unicast_stream->stream;
			}
		}
	}

	return NULL;
}

static int lc3_config(struct bt_conn *conn, const struct bt_bap_ep *ep, enum bt_audio_dir dir,
		      const struct bt_audio_codec_cfg *codec_cfg, struct bt_bap_stream **stream,
		      struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	printk("ASE Codec Config: conn %p ep %p dir %s\n", conn, ep, dir_str(dir));

	print_codec_cfg(codec_cfg);

	*stream = stream_alloc(dir);
	if (*stream == NULL) {
		printk("No %s streams available\n", dir_str(dir));
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_NO_MEM, BT_BAP_ASCS_REASON_NONE);

		return -ENOMEM;
	}

	printk("ASE Codec Config stream %p\n", *stream);

	*pref = qos_pref;

	return 0;
}

static int lc3_reconfig(struct bt_bap_stream *stream, enum bt_audio_dir dir,
			const struct bt_audio_codec_cfg *codec_cfg,
			struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	printk("ASE Codec Reconfig: stream %p dir %s\n", stream, dir_str(dir));
	print_codec_cfg(codec_cfg);
	*pref = qos_pref;

	return 0;
}

static int lc3_qos(struct bt_bap_stream *stream, const struct bt_bap_qos_cfg *qos,
		   struct bt_bap_ascs_rsp *rsp)
{
	printk("QoS: stream %p qos %p\n", stream, qos);

	print_qos(qos);

	return 0;
}

static int lc3_enable(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len,
		      struct bt_bap_ascs_rsp *rsp)
{
	printk("Enable: stream %p meta_len %zu\n", stream, meta_len);

	bt_audio_data_parse(meta, meta_len, print_cb, "meta");

	return 0;
}

static int lc3_start(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	printk("Start: stream %p\n", stream);

	return 0;
}

static int lc3_metadata(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len,
			struct bt_bap_ascs_rsp *rsp)
{
	printk("Metadata: stream %p meta_len %zu\n", stream, meta_len);

	/* The metadata is only logged. In particular, the CCID list is not validated as the
	 * phone will typically refer to its own (G)TBS, which this sample does not discover.
	 */
	bt_audio_data_parse(meta, meta_len, print_cb, "meta");

	return 0;
}

static int lc3_disable(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	printk("Disable: stream %p\n", stream);

	return 0;
}

static int lc3_stop(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	printk("Stop: stream %p\n", stream);

	return 0;
}

static int lc3_release(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	printk("Release: stream %p\n", stream);

	return 0;
}

static struct bt_bap_unicast_server_register_param param = {
	CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT,
	CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT
};

static const struct bt_bap_unicast_server_cb unicast_server_cb = {
	.config = lc3_config,
	.reconfig = lc3_reconfig,
	.qos = lc3_qos,
	.enable = lc3_enable,
	.start = lc3_start,
	.metadata = lc3_metadata,
	.disable = lc3_disable,
	.stop = lc3_stop,
	.release = lc3_release,
};

static void stream_recv(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info,
			struct net_buf *buf)
{
	struct unicast_stream *unicast_stream =
		CONTAINER_OF(stream, struct unicast_stream, stream);

	iso_rx_log("CIS", stream, &unicast_stream->rx_stats, info, buf);
	audio_out_recv(stream, info, buf);
}

static void stream_sent(struct bt_bap_stream *stream)
{
	struct unicast_stream *unicast_stream =
		CONTAINER_OF(stream, struct unicast_stream, stream);

	unicast_stream->tx_cnt++;
	if (unicast_stream->tx_cnt == 1U || (unicast_stream->tx_cnt % TX_LOG_INTERVAL) == 0U) {
		printk("[CIS] stream %p sent %zu SDUs\n", stream, unicast_stream->tx_cnt);
	}
}

static void stream_enabled(struct bt_bap_stream *stream)
{
	struct bt_bap_ep_info ep_info;
	int err;

	err = bt_bap_ep_get_info(stream->ep, &ep_info);
	if (err != 0) {
		printk("Failed to get ep info: %d\n", err);
		return;
	}

	printk("Enabled %s stream %p\n", dir_str(ep_info.dir), stream);

	/* The unicast server is responsible for starting the sink streams */
	if (ep_info.dir == BT_AUDIO_DIR_SINK) {
		/* Automatically do the receiver start ready operation */
		err = bt_bap_stream_start(stream);

		if (err != 0) {
			printk("Failed to start stream %p: %d\n", stream, err);
		}
	}
}

static void stream_started(struct bt_bap_stream *stream)
{
	struct unicast_stream *unicast_stream =
		CONTAINER_OF(stream, struct unicast_stream, stream);

	printk("Unicast stream %p started\n", stream);

	iso_rx_reset(&unicast_stream->rx_stats);
	unicast_stream->tx_seq_num = 0U;
	unicast_stream->tx_cnt = 0U;

	/* Play the received audio; the source (microphone) streams only send dummy SDUs */
	if (IS_ARRAY_ELEMENT(sink_streams, unicast_stream)) {
		(void)audio_out_stream_start(AUDIO_OUT_SRC_CIS, stream);
	}
}

static void stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
	printk("Unicast stream %p stopped with reason 0x%02X\n", stream, reason);

	audio_out_stream_stop(stream);
}

static void stream_released(struct bt_bap_stream *stream)
{
	printk("Unicast stream %p released\n", stream);

	/* In case the stream was released without being stopped, e.g. on disconnect */
	audio_out_stream_stop(stream);
}

static struct bt_bap_stream_ops stream_ops = {
	.enabled = stream_enabled,
	.started = stream_started,
	.stopped = stream_stopped,
	.released = stream_released,
	.recv = stream_recv,
	.sent = stream_sent,
};

static bool stream_is_streaming(const struct bt_bap_stream *stream)
{
	struct bt_bap_ep_info ep_info;

	if (stream->ep == NULL) {
		return false;
	}

	if (bt_bap_ep_get_info(stream->ep, &ep_info) != 0) {
		return false;
	}

	return ep_info.state == BT_BAP_EP_STATE_STREAMING;
}

/* Sends dummy (all zero) SDUs on the source streams. The rate is limited by the number of TX
 * buffers, which are released when the controller has sent the SDU.
 */
static void tx_thread_func(void *arg1, void *arg2, void *arg3)
{
	NET_BUF_POOL_FIXED_DEFINE(tx_pool, CONFIG_BT_ISO_TX_BUF_COUNT,
				  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
				  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);
	static const uint8_t data[CONFIG_BT_ISO_TX_MTU];

	while (true) {
		bool sent = false;

		ARRAY_FOR_EACH_PTR(source_streams, unicast_stream) {
			struct bt_bap_stream *stream = &unicast_stream->stream;
			struct net_buf *buf;
			int err;

			if (!stream_is_streaming(stream) || stream->qos == NULL) {
				continue;
			}

			buf = net_buf_alloc(&tx_pool, K_FOREVER);
			net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);
			net_buf_add_mem(buf, data, MIN(stream->qos->sdu, sizeof(data)));

			err = bt_bap_stream_send(stream, buf, unicast_stream->tx_seq_num);
			if (err == 0) {
				unicast_stream->tx_seq_num++;
				sent = true;
			} else {
				printk("Unable to send on stream %p: %d\n", stream, err);
				net_buf_unref(buf);
			}
		}

		if (!sent) {
			/* Nothing is streaming, or sending failed; retry with a delay */
			k_sleep(K_MSEC(100));
		}
	}
}

int bap_unicast_sr_init(void)
{
	int err;

	err = bt_bap_unicast_server_register(&param);
	if (err != 0) {
		printk("Failed to register BAP unicast server (err %d)\n", err);
		return err;
	}

	err = bt_bap_unicast_server_register_cb(&unicast_server_cb);
	if (err != 0) {
		printk("Failed to register BAP unicast server callbacks (err %d)\n", err);
		return err;
	}

	ARRAY_FOR_EACH_PTR(sink_streams, unicast_stream) {
		bt_bap_stream_cb_register(&unicast_stream->stream, &stream_ops);
	}

	ARRAY_FOR_EACH_PTR(source_streams, unicast_stream) {
		bt_bap_stream_cb_register(&unicast_stream->stream, &stream_ops);
	}

	if (IS_ENABLED(CONFIG_BT_ASCS_ASE_SRC)) {
		static K_KERNEL_STACK_DEFINE(tx_thread_stack, 1024);
		static struct k_thread tx_thread;

		k_thread_create(&tx_thread, tx_thread_stack, K_KERNEL_STACK_SIZEOF(tx_thread_stack),
				tx_thread_func, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
		k_thread_name_set(&tx_thread, "TX thread");
	}

	return 0;
}
