/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "nrf54l15_audio_headset.h"

#define ISO_RX_HEX_LEN 8U

void iso_rx_reset(struct iso_rx_stats *stats)
{
	(void)memset(stats, 0, sizeof(*stats));
}

void iso_rx_log(const char *tag, const struct bt_bap_stream *stream, struct iso_rx_stats *stats,
		const struct bt_iso_recv_info *info, const struct net_buf *buf)
{
	bool do_log = false;

	stats->recv_cnt++;
	if (stats->recv_cnt == 1U || (stats->recv_cnt % ISO_RX_LOG_INTERVAL) == 0U) {
		do_log = true;
	}

	if ((info->flags & BT_ISO_FLAGS_ERROR) != 0U) {
		stats->err_cnt++;
		do_log = true;
	}

	if ((info->flags & BT_ISO_FLAGS_LOST) != 0U) {
		stats->lost_cnt++;
		do_log = true;
	}

	if ((info->flags & BT_ISO_FLAGS_VALID) != 0U) {
		if (buf->len == 0U) {
			stats->empty_cnt++;
		} else {
			stats->valid_cnt++;
		}
	}

	if (!do_log) {
		return;
	}

	printk("[%s] stream %p #%zu len %u seq %u ts %u flags 0x%02X | valid %zu empty %zu "
	       "lost %zu err %zu | data:",
	       tag, stream, stats->recv_cnt, buf->len, info->seq_num, info->ts, info->flags,
	       stats->valid_cnt, stats->empty_cnt, stats->lost_cnt, stats->err_cnt);

	for (size_t i = 0U; i < MIN(buf->len, ISO_RX_HEX_LEN); i++) {
		printk(" %02x", buf->data[i]);
	}

	printk("\n");
}
