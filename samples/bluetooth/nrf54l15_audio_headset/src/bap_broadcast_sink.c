/*
 * Copyright (c) 2022-2026 Nordic Semiconductor ASA
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/autoconf.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/assigned_numbers.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/tmap.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

#include "nrf54l15_audio_headset.h"

#define PA_SYNC_SKIP                      5
#define PA_SYNC_INTERVAL_TO_TIMEOUT_RATIO 20 /* Set the timeout relative to interval */

enum broadcast_flag {
	FLAG_BROADCAST_CODE_REQUIRED,
	FLAG_BROADCAST_CODE_RECEIVED,
	FLAG_BROADCAST_SYNCABLE,
	FLAG_BROADCAST_SYNCING,
	FLAG_BROADCAST_SYNCED,
	FLAG_BASE_RECEIVED,
	FLAG_PA_SYNCING,
	FLAG_PA_SYNCED,
	FLAG_SCANNING,
	FLAG_NUM,
};
ATOMIC_DEFINE(flags, FLAG_NUM);

static struct broadcast_stream {
	struct bt_bap_stream stream;
	struct iso_rx_stats rx_stats;
} broadcast_streams[CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT];

static struct broadcast_sink {
	const struct bt_bap_scan_delegator_recv_state *req_recv_state;
	uint8_t sink_broadcast_code[BT_ISO_BROADCAST_CODE_SIZE];
	struct bt_bap_broadcast_sink *bap_broadcast_sink;
	struct bt_le_per_adv_sync *pa_sync;
	uint8_t received_base[UINT8_MAX];
	/* BIS indexes requested by the Broadcast Assistant; 0 if no sync is requested */
	uint32_t requested_bis_sync;
	uint32_t broadcast_id;
} broadcast_sink;

static uint16_t interval_to_sync_timeout(uint16_t pa_interval)
{
	uint16_t pa_timeout;

	if (pa_interval == BT_BAP_PA_INTERVAL_UNKNOWN) {
		/* Use maximum value to maximize chance of success */
		pa_timeout = BT_GAP_PER_ADV_MAX_TIMEOUT;
	} else {
		uint32_t interval_us;
		uint32_t timeout;

		/* Add retries and convert to unit in 10's of ms */
		interval_us = BT_GAP_PER_ADV_INTERVAL_TO_US(pa_interval);
		timeout = BT_GAP_US_TO_PER_ADV_SYNC_TIMEOUT(interval_us) *
			  PA_SYNC_INTERVAL_TO_TIMEOUT_RATIO;

		/* Enforce restraints */
		pa_timeout = CLAMP(timeout, BT_GAP_PER_ADV_MIN_TIMEOUT, BT_GAP_PER_ADV_MAX_TIMEOUT);
	}

	return pa_timeout;
}

/** Starts scanning if it passes a series of check to determine if we are in the right state */
static int check_start_scan(void)
{
	int err;

	if (!IS_ENABLED(CONFIG_NRF54L15_AUDIO_HEADSET_SCAN_SELF)) {
		return -ENOTSUP;
	}

	if (atomic_test_bit(flags, FLAG_SCANNING) || atomic_test_bit(flags, FLAG_PA_SYNCING) ||
	    atomic_test_bit(flags, FLAG_PA_SYNCED) ||
	    atomic_test_bit(flags, FLAG_BROADCAST_SYNCED)) {
		return -EALREADY;
	}

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, NULL);
	if (err != 0) {
		printk("Unable to start scan for TMAP BMS: %d\n", err);

		return err;
	}

	atomic_set_bit(flags, FLAG_SCANNING);
	printk("Scanning for TMAP BMS\n");

	return 0;
}

static void broadcast_stop_work_handler(struct k_work *work)
{
	int err;

	if (broadcast_sink.requested_bis_sync != 0U ||
	    !atomic_test_bit(flags, FLAG_BROADCAST_SYNCED)) {
		return;
	}

	printk("BIS sync is no longer requested, stopping sync\n");

	err = bt_bap_broadcast_sink_stop(broadcast_sink.bap_broadcast_sink);
	if (err != 0) {
		printk("Failed to stop Broadcast Sink: %d\n", err);
	}
}

static K_WORK_DEFINE(broadcast_stop_work, broadcast_stop_work_handler);

static void broadcast_stream_started_cb(struct bt_bap_stream *stream)
{
	struct broadcast_stream *broadcast_stream =
		CONTAINER_OF(stream, struct broadcast_stream, stream);

	printk("Broadcast stream %p started\n", stream);

	iso_rx_reset(&broadcast_stream->rx_stats);
	atomic_clear_bit(flags, FLAG_BROADCAST_SYNCING);
	atomic_set_bit(flags, FLAG_BROADCAST_SYNCED);

	if (broadcast_sink.requested_bis_sync == 0U) {
		/* The Broadcast Assistant withdrew the BIS sync request while we were syncing and
		 * the sync could not be cancelled. The BIG cannot be stopped from this callback.
		 */
		k_work_submit(&broadcast_stop_work);
		return;
	}

	(void)audio_out_stream_start(AUDIO_OUT_SRC_BIS, stream);
}

static void broadcast_stream_stopped_cb(struct bt_bap_stream *stream, uint8_t reason)
{
	printk("Broadcast stream %p stopped with reason 0x%02X\n", stream, reason);

	atomic_clear_bit(flags, FLAG_BROADCAST_SYNCING);
	atomic_clear_bit(flags, FLAG_BROADCAST_SYNCED);

	audio_out_stream_stop(stream);
}

static void broadcast_stream_recv_cb(struct bt_bap_stream *stream,
				     const struct bt_iso_recv_info *info, struct net_buf *buf)
{
	struct broadcast_stream *broadcast_stream =
		CONTAINER_OF(stream, struct broadcast_stream, stream);

	iso_rx_log("BIS", stream, &broadcast_stream->rx_stats, info, buf);
	audio_out_recv(stream, info, buf);
}

static int create_broadcast_sink(void)
{
	int err;

	if (broadcast_sink.bap_broadcast_sink != NULL) {
		return -EALREADY;
	}

	printk("Creating broadcast sink for broadcast ID 0x%06X\n", broadcast_sink.broadcast_id);

	err = bt_bap_broadcast_sink_create(broadcast_sink.pa_sync, broadcast_sink.broadcast_id,
					   &broadcast_sink.bap_broadcast_sink);
	if (err != 0) {
		printk("Failed to create broadcast sink: %d\n", err);

		return err;
	}

	return 0;
}

static void delete_broadcast_sink(void)
{
	int err;

	if (broadcast_sink.bap_broadcast_sink == NULL) {
		return;
	}

	err = bt_bap_broadcast_sink_delete(broadcast_sink.bap_broadcast_sink);
	if (err != 0) {
		printk("Failed to delete broadcast sink: %d\n", err);
	} else {
		broadcast_sink.bap_broadcast_sink = NULL;
	}
}

/** Selects at most one BIS index per stream we have, starting from the lowest index */
static uint32_t select_bis_indexes(uint32_t candidates)
{
	uint32_t selected = 0U;
	size_t cnt = 0U;

	for (uint8_t i = BT_ISO_BIS_INDEX_MIN; i <= BT_ISO_BIS_INDEX_MAX; i++) {
		if (cnt == ARRAY_SIZE(broadcast_streams)) {
			break;
		}

		if ((candidates & BT_ISO_BIS_INDEX_BIT(i)) != 0U) {
			selected |= BT_ISO_BIS_INDEX_BIT(i);
			cnt++;
		}
	}

	return selected;
}

struct subgroup_bis_data {
	uint32_t requested;
	uint32_t candidates;
};

static bool subgroup_bis_cb(const struct bt_bap_base_subgroup *subgroup, void *user_data)
{
	struct subgroup_bis_data *data = user_data;
	uint32_t subgroup_bis;

	if (bt_bap_base_subgroup_get_bis_indexes(subgroup, &subgroup_bis) != 0) {
		return true;
	}

	if (data->requested != BT_BAP_BIS_SYNC_NO_PREF) {
		subgroup_bis &= data->requested;
	}

	if (subgroup_bis == 0U) {
		return true;
	}

	data->candidates = subgroup_bis;

	return false;
}

/** Returns the requested BIS indexes of the first subgroup that contains any of them. Only BISes
 * of a single subgroup are synced, as they are decoded with the same codec configuration and
 * carry the channels of the same audio (rather than e.g. another language).
 */
static uint32_t subgroup_bis_get(const struct bt_bap_base *base, uint32_t requested)
{
	struct subgroup_bis_data data = {
		.requested = requested,
		.candidates = 0U,
	};

	(void)bt_bap_base_foreach_subgroup(base, subgroup_bis_cb, &data);

	return data.candidates;
}

/** Performs a series of checks to see if we are ready to sync the broadcast sink */
static void check_sync_broadcast(void)
{
	struct bt_bap_stream *sync_streams[ARRAY_SIZE(broadcast_streams)];
	uint32_t candidates;
	uint32_t sync_bitfield;
	int err;

	if (broadcast_sink.requested_bis_sync == 0U ||
	    broadcast_sink.bap_broadcast_sink == NULL ||
	    !atomic_test_bit(flags, FLAG_PA_SYNCED) ||
	    !atomic_test_bit(flags, FLAG_BASE_RECEIVED) ||
	    !atomic_test_bit(flags, FLAG_BROADCAST_SYNCABLE) ||
	    atomic_test_bit(flags, FLAG_BROADCAST_SYNCED) ||
	    atomic_test_bit(flags, FLAG_BROADCAST_SYNCING)) {
		return;
	}

	if (atomic_test_bit(flags, FLAG_BROADCAST_CODE_REQUIRED) &&
	    !atomic_test_bit(flags, FLAG_BROADCAST_CODE_RECEIVED)) {
		/* Wait for the Broadcast Assistant to write the Broadcast Code */
		return;
	}

	candidates = subgroup_bis_get((const struct bt_bap_base *)broadcast_sink.received_base,
				      broadcast_sink.requested_bis_sync);
	sync_bitfield = select_bis_indexes(candidates);
	if (sync_bitfield == 0U) {
		printk("No BIS to sync to (requested 0x%08X)\n", broadcast_sink.requested_bis_sync);

		return;
	}

	if (sync_bitfield != candidates) {
		printk("Can only sync to %zu BIS: syncing to 0x%08X of 0x%08X\n",
		       ARRAY_SIZE(broadcast_streams), sync_bitfield, candidates);
	}

	ARRAY_FOR_EACH(sync_streams, i) {
		sync_streams[i] = &broadcast_streams[i].stream;
	}

	printk("Syncing to broadcast with bitfield 0x%08X\n", sync_bitfield);

	err = bt_bap_broadcast_sink_sync(broadcast_sink.bap_broadcast_sink, sync_bitfield,
					 sync_streams, broadcast_sink.sink_broadcast_code);
	if (err != 0) {
		printk("Failed to sync the broadcast sink: %d\n", err);
	} else {
		atomic_set_bit(flags, FLAG_BROADCAST_SYNCING);
	}
}

static void base_recv_cb(struct bt_bap_broadcast_sink *sink, const struct bt_bap_base *base,
			 size_t base_size)
{
	memcpy(broadcast_sink.received_base, base,
	       MIN(base_size, sizeof(broadcast_sink.received_base)));

	if (!atomic_test_and_set_bit(flags, FLAG_BASE_RECEIVED)) {
		printk("BASE received\n");

		check_sync_broadcast();
	}
}

static void syncable_cb(struct bt_bap_broadcast_sink *sink, const struct bt_iso_biginfo *biginfo)
{
	atomic_set_bit_to(flags, FLAG_BROADCAST_CODE_REQUIRED, biginfo->encryption);

	if (!atomic_test_and_set_bit(flags, FLAG_BROADCAST_SYNCABLE)) {
		printk("BIGInfo received: %u BIS, encryption %s\n", biginfo->num_bis,
		       biginfo->encryption ? "yes" : "no");
	}

	/* BIGInfo is reported periodically while we are not synced to the BIG, so this also
	 * retries the sync if the BIG was lost while the PA is still synced
	 */
	check_sync_broadcast();
}

static void sink_started_cb(struct bt_bap_broadcast_sink *sink)
{
	printk("Broadcast sink started\n");
}

static void sink_stopped_cb(struct bt_bap_broadcast_sink *sink, uint8_t reason)
{
	printk("Broadcast sink stopped with reason 0x%02X\n", reason);

	atomic_clear_bit(flags, FLAG_BROADCAST_SYNCING);
	atomic_clear_bit(flags, FLAG_BROADCAST_SYNCED);

	if (reason == BT_HCI_ERR_TERM_DUE_TO_MIC_FAIL) {
		/* Wrong Broadcast Code: Wait for the Broadcast Assistant to provide a new one */
		atomic_clear_bit(flags, FLAG_BROADCAST_CODE_RECEIVED);
	}

	/* The sink is kept while the PA is synced so that we can sync to the BIG again */
	if (broadcast_sink.pa_sync == NULL) {
		delete_broadcast_sink();
		(void)check_start_scan();
	}
}

static void pa_timer_handler(struct k_work *work)
{
	atomic_clear_bit(flags, FLAG_PA_SYNCING);

	if (broadcast_sink.pa_sync != NULL) {
		int err;

		err = bt_le_per_adv_sync_delete(broadcast_sink.pa_sync);
		if (err != 0) {
			printk("Failed to delete PA sync: %d\n", err);
		}

		broadcast_sink.pa_sync = NULL;
	}

	if (broadcast_sink.req_recv_state != NULL) {
		enum bt_bap_pa_state pa_state;

		if (broadcast_sink.req_recv_state->pa_sync_state == BT_BAP_PA_STATE_INFO_REQ) {
			pa_state = BT_BAP_PA_STATE_NO_PAST;
		} else {
			pa_state = BT_BAP_PA_STATE_FAILED;
		}

		bt_bap_scan_delegator_set_pa_state(broadcast_sink.req_recv_state->src_id, pa_state);
	}

	printk("PA sync timeout\n");

	(void)check_start_scan();
}

static K_WORK_DELAYABLE_DEFINE(pa_timer, pa_timer_handler);

static int pa_sync_with_past(struct bt_conn *conn,
			     const struct bt_bap_scan_delegator_recv_state *recv_state,
			     uint16_t pa_interval)
{
	struct bt_le_per_adv_sync_transfer_param param = {0};
	int err;

	param.skip = PA_SYNC_SKIP;
	param.timeout = interval_to_sync_timeout(pa_interval);

	err = bt_le_per_adv_sync_transfer_subscribe(conn, &param);
	if (err != 0) {
		printk("Could not do PAST subscribe: %d\n", err);

		return err;
	}

	err = bt_bap_scan_delegator_set_pa_state(recv_state->src_id, BT_BAP_PA_STATE_INFO_REQ);
	if (err != 0) {
		printk("Failed to set PA state to BT_BAP_PA_STATE_INFO_REQ: %d\n", err);

		return err;
	}

	k_work_reschedule(&pa_timer, K_MSEC(param.timeout * 10));

	return 0;
}

static int pa_sync_without_past(const bt_addr_le_t *addr, uint8_t adv_sid, uint16_t pa_interval)
{
	struct bt_le_per_adv_sync_param param = {0};
	int err;

	bt_addr_le_copy(&param.addr, addr);
	param.options = BT_LE_PER_ADV_SYNC_OPT_FILTER_DUPLICATE;
	param.sid = adv_sid;
	param.skip = PA_SYNC_SKIP;
	param.timeout = interval_to_sync_timeout(pa_interval);

	err = bt_le_per_adv_sync_create(&param, &broadcast_sink.pa_sync);
	if (err != 0) {
		printk("Failed to create PA sync: %d\n", err);

		return err;
	}

	k_work_reschedule(&pa_timer, K_MSEC(param.timeout * 10));

	return 0;
}

static int pa_sync_req_cb(struct bt_conn *conn,
			  const struct bt_bap_scan_delegator_recv_state *recv_state,
			  bool past_avail, uint16_t pa_interval)
{
	int err;

	printk("Received request to sync to PA (PAST %savailable): %u\n", past_avail ? "" : "not ",
	       recv_state->pa_sync_state);

	broadcast_sink.req_recv_state = recv_state;

	if (recv_state->pa_sync_state == BT_BAP_PA_STATE_SYNCED ||
	    recv_state->pa_sync_state == BT_BAP_PA_STATE_INFO_REQ ||
	    broadcast_sink.pa_sync != NULL) {
		/* Already syncing */
		printk("Rejecting PA sync request because we are already syncing or synced\n");

		return -EALREADY;
	}

	if (atomic_test_bit(flags, FLAG_SCANNING)) {
		/* The Broadcast Assistant decides what to sync to; stop looking for a TMAP BMS */
		err = bt_le_scan_stop();
		if (err != 0) {
			printk("Unable to stop scanning: %d\n", err);
		} else {
			atomic_clear_bit(flags, FLAG_SCANNING);
		}
	}

	if (IS_ENABLED(CONFIG_BT_PER_ADV_SYNC_TRANSFER_RECEIVER) && past_avail) {
		err = pa_sync_with_past(conn, recv_state, pa_interval);
		if (err != 0) {
			return err;
		}

		printk("Syncing with PAST\n");
	} else {
		err = pa_sync_without_past(&recv_state->addr, recv_state->adv_sid, pa_interval);
		if (err != 0) {
			return err;
		}

		printk("Syncing without PAST\n");
	}

	broadcast_sink.broadcast_id = recv_state->broadcast_id;
	atomic_set_bit(flags, FLAG_PA_SYNCING);

	return 0;
}

static int pa_sync_term_req_cb(struct bt_conn *conn,
			       const struct bt_bap_scan_delegator_recv_state *recv_state)
{
	int err;

	printk("Received request to terminate PA sync\n");

	broadcast_sink.req_recv_state = recv_state;
	k_work_cancel_delayable(&pa_timer);
	atomic_clear_bit(flags, FLAG_PA_SYNCING);

	if (broadcast_sink.pa_sync == NULL) {
		/* E.g. still waiting for PAST */
		return bt_bap_scan_delegator_set_pa_state(recv_state->src_id,
							   BT_BAP_PA_STATE_NOT_SYNCED);
	}

	/* This will trigger bap_pa_sync_terminated_cb */
	err = bt_le_per_adv_sync_delete(broadcast_sink.pa_sync);
	if (err != 0) {
		printk("Failed to delete PA sync: %d\n", err);

		return err;
	}

	broadcast_sink.pa_sync = NULL;

	return 0;
}

static void broadcast_code_cb(struct bt_conn *conn,
			      const struct bt_bap_scan_delegator_recv_state *recv_state,
			      const uint8_t broadcast_code[BT_ISO_BROADCAST_CODE_SIZE])
{
	printk("Broadcast code received for %p\n", recv_state);

	broadcast_sink.req_recv_state = recv_state;

	(void)memcpy(broadcast_sink.sink_broadcast_code, broadcast_code,
		     BT_ISO_BROADCAST_CODE_SIZE);

	atomic_set_bit(flags, FLAG_BROADCAST_CODE_RECEIVED);

	/* The code is typically written after the BIGInfo has been received, so this may be the
	 * last missing piece
	 */
	check_sync_broadcast();
}

static void recv_state_updated_cb(struct bt_conn *conn,
				  const struct bt_bap_scan_delegator_recv_state *recv_state)
{
	printk("Receive state %u: PA sync state %u, encryption state %u", recv_state->src_id,
	       recv_state->pa_sync_state, recv_state->encrypt_state);

	for (uint8_t i = 0U; i < recv_state->num_subgroups; i++) {
		printk(", subgroup %u BIS sync 0x%08X", i, recv_state->subgroups[i].bis_sync);
	}

	printk("\n");
}

static int remove_source_cb(struct bt_conn *conn, uint8_t src_id)
{
	printk("Removing broadcast source %u\n", src_id);

	if (broadcast_sink.req_recv_state != NULL &&
	    broadcast_sink.req_recv_state->src_id == src_id) {
		broadcast_sink.req_recv_state = NULL;
	}

	broadcast_sink.requested_bis_sync = 0U;

	/* The scan delegator only removes sources that are neither PA nor BIS synced, so the
	 * broadcast sink of the source is idle and can be deleted
	 */
	if (broadcast_sink.pa_sync != NULL) {
		int err;

		/* This will trigger bap_pa_sync_terminated_cb, which deletes the broadcast sink */
		err = bt_le_per_adv_sync_delete(broadcast_sink.pa_sync);
		if (err != 0) {
			printk("Failed to delete PA sync: %d\n", err);
		}
	} else if (!atomic_test_bit(flags, FLAG_BROADCAST_SYNCED) &&
		   !atomic_test_bit(flags, FLAG_BROADCAST_SYNCING)) {
		delete_broadcast_sink();
	}

	return 0;
}

static uint32_t get_req_bis_sync(const uint32_t bis_sync_req[CONFIG_BT_BAP_BASS_MAX_SUBGROUPS])
{
	uint32_t bis_sync = 0U;

	for (int i = 0; i < CONFIG_BT_BAP_BASS_MAX_SUBGROUPS; i++) {
		bis_sync |= bis_sync_req[i];
	}

	return bis_sync;
}

static int bis_sync_req_cb(struct bt_conn *conn,
			   const struct bt_bap_scan_delegator_recv_state *recv_state,
			   const uint32_t bis_sync_req[CONFIG_BT_BAP_BASS_MAX_SUBGROUPS])
{
	const uint32_t new_bis_sync_req = get_req_bis_sync(bis_sync_req);
	const bool big_active = atomic_test_bit(flags, FLAG_BROADCAST_SYNCED) ||
				atomic_test_bit(flags, FLAG_BROADCAST_SYNCING);

	printk("BIS sync request received for %p: 0x%08X\n", recv_state, new_bis_sync_req);

	broadcast_sink.req_recv_state = recv_state;

	if (big_active && broadcast_sink.requested_bis_sync == new_bis_sync_req) {
		printk("Already syncing or synced to the requested BISes; ignoring\n");

		return 0; /* no op */
	}

	if (big_active) {
		/* The requested BIS sync has changed, e.g. to 0 to stop the reception. Stop the
		 * current sync (or cancel a pending one). The stream stopped callback will be
		 * called as part of this, and we do not need to wait for any events from the
		 * controller. Thus, when this returns, the broadcast sink is stopped.
		 */
		int err;

		printk("Stopping current sync\n");

		err = bt_bap_broadcast_sink_stop(broadcast_sink.bap_broadcast_sink);
		if (err != 0) {
			/* If the BIG still gets synced, broadcast_stream_started_cb() stops it */
			printk("Failed to stop Broadcast Sink: %d\n", err);
		}
	}

	/* Mirror the request of the scan delegator, which only calls this callback again when
	 * the request changes. A request of 0 thus stays in effect until the next request.
	 */
	broadcast_sink.requested_bis_sync = new_bis_sync_req;
	check_sync_broadcast();

	return 0;
}

static void bap_pa_sync_synced_cb(struct bt_le_per_adv_sync *sync,
				  struct bt_le_per_adv_sync_synced_info *info)
{
	int err;

	/* Accept the sync if it is the one we created, if it was transferred to us (PAST) while we
	 * are waiting for it, or if it matches the requested receive state
	 */
	if (!(sync == broadcast_sink.pa_sync ||
	      (atomic_test_bit(flags, FLAG_PA_SYNCING) && info->conn != NULL) ||
	      (broadcast_sink.req_recv_state != NULL &&
	       bt_addr_le_eq(info->addr, &broadcast_sink.req_recv_state->addr) &&
	       info->sid == broadcast_sink.req_recv_state->adv_sid))) {
		return;
	}

	printk("PA sync %p synced for broadcast sink (%s)\n", (void *)sync,
	       info->conn != NULL ? "PAST" : "no PAST");

	if (broadcast_sink.pa_sync == NULL) {
		broadcast_sink.pa_sync = sync;
	}

	k_work_cancel_delayable(&pa_timer);
	atomic_set_bit(flags, FLAG_PA_SYNCED);
	atomic_clear_bit(flags, FLAG_PA_SYNCING);

	if (atomic_test_bit(flags, FLAG_SCANNING)) {
		err = bt_le_scan_stop();
		if (err != 0) {
			printk("Unable to stop scanning: %d\n", err);
		} else {
			atomic_clear_bit(flags, FLAG_SCANNING);
		}
	}

	err = create_broadcast_sink();
	if (err == 0) {
		check_sync_broadcast();
	}
}

static void bap_pa_sync_terminated_cb(struct bt_le_per_adv_sync *sync,
				      const struct bt_le_per_adv_sync_term_info *info)
{
	if (sync != broadcast_sink.pa_sync) {
		return;
	}

	printk("PA sync %p lost with reason 0x%02X\n", (void *)sync, info->reason);

	/* Without PA we cannot sync to any new BIG - Clear data. The BIS sync request is kept: the
	 * BIG may still be synced, and the request remains in effect if the PA is synced again.
	 */
	broadcast_sink.pa_sync = NULL;
	k_work_cancel_delayable(&pa_timer);
	atomic_clear_bit(flags, FLAG_BROADCAST_SYNCABLE);
	atomic_clear_bit(flags, FLAG_PA_SYNCED);
	atomic_clear_bit(flags, FLAG_PA_SYNCING);
	atomic_clear_bit(flags, FLAG_BASE_RECEIVED);
	atomic_clear_bit(flags, FLAG_BROADCAST_CODE_REQUIRED);
	atomic_clear_bit(flags, FLAG_BROADCAST_CODE_RECEIVED);

	if (broadcast_sink.req_recv_state != NULL) {
		int err;

		err = bt_bap_scan_delegator_set_pa_state(broadcast_sink.req_recv_state->src_id,
							 BT_BAP_PA_STATE_NOT_SYNCED);
		if (err != 0) {
			printk("Failed to set PA state to BT_BAP_PA_STATE_NOT_SYNCED: %d\n", err);
		}
	}

	/* If the BIG is still synced, the sink is deleted once it stops */
	if (!atomic_test_bit(flags, FLAG_BROADCAST_SYNCED) &&
	    !atomic_test_bit(flags, FLAG_BROADCAST_SYNCING)) {
		delete_broadcast_sink();
		(void)check_start_scan();
	}
}

struct scan_result {
	uint32_t broadcast_id;
	bool tmap_bms_found;
};

static bool scan_check_broadcast(struct bt_data *data, void *user_data)
{
	struct scan_result *result = user_data;
	struct bt_uuid_16 adv_uuid;

	if (data->type != BT_DATA_SVC_DATA16) {
		return true;
	}

	if (data->data_len < BT_UUID_SIZE_16) {
		return true;
	}

	if (!bt_uuid_create(&adv_uuid.uuid, data->data, BT_UUID_SIZE_16)) {
		return true;
	}

	if (bt_uuid_cmp(&adv_uuid.uuid, BT_UUID_BROADCAST_AUDIO) == 0) {
		if (data->data_len >= BT_UUID_SIZE_16 + BT_AUDIO_BROADCAST_ID_SIZE) {
			result->broadcast_id = sys_get_le24(data->data + BT_UUID_SIZE_16);
		}

		return true;
	}

	if (bt_uuid_cmp(&adv_uuid.uuid, BT_UUID_TMAS) == 0) {
		uint16_t peer_tmap_role;

		if (data->data_len < BT_UUID_SIZE_16 + sizeof(peer_tmap_role)) {
			return false;
		}

		peer_tmap_role = sys_get_le16(data->data + BT_UUID_SIZE_16);
		if ((peer_tmap_role & BT_TMAP_ROLE_BMS) != 0U) {
			result->tmap_bms_found = true;
		}

		return true;
	}

	return true;
}

static void broadcast_scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *ad)
{
	struct bt_le_per_adv_sync_param param = {0};
	struct scan_result result = {
		.broadcast_id = BT_BAP_INVALID_BROADCAST_ID,
		.tmap_bms_found = false,
	};
	int err;

	if (!atomic_test_bit(flags, FLAG_SCANNING) || atomic_test_bit(flags, FLAG_PA_SYNCING) ||
	    atomic_test_bit(flags, FLAG_PA_SYNCED)) {
		/* If we are already synced or syncing, we do not care about scan reports */
		return;
	}

	/* We are only interested in non-connectable periodic advertisers */
	if ((info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) != 0U || info->interval == 0U) {
		return;
	}

	bt_data_parse(ad, scan_check_broadcast, &result);
	if (result.broadcast_id == BT_BAP_INVALID_BROADCAST_ID || !result.tmap_bms_found) {
		return;
	}

	printk("Found TMAP BMS with broadcast ID 0x%06X and addr %s and sid 0x%02X\n",
	       result.broadcast_id, bt_addr_le_str(info->addr), info->sid);

	bt_addr_le_copy(&param.addr, info->addr);
	param.options = BT_LE_PER_ADV_SYNC_OPT_FILTER_DUPLICATE;
	param.sid = info->sid;
	param.skip = PA_SYNC_SKIP;
	param.timeout = interval_to_sync_timeout(info->interval);

	err = bt_le_per_adv_sync_create(&param, &broadcast_sink.pa_sync);
	if (err != 0) {
		printk("Failed to create PA sync: %d\n", err);

		return;
	}

	printk("Syncing without PAST from scan\n");

	broadcast_sink.broadcast_id = result.broadcast_id;
	atomic_set_bit(flags, FLAG_PA_SYNCING);
	k_work_reschedule(&pa_timer, K_MSEC(param.timeout * 10));

	/* Since we are scanning ourselves, we consider this as broadcast sync has been
	 * requested
	 */
	broadcast_sink.requested_bis_sync = BT_BAP_BIS_SYNC_NO_PREF;
}

void bap_broadcast_sink_stop_bis(void)
{
	int err;

	/* Do not sync to the BIG again by ourselves; the Broadcast Assistant can request the BIS
	 * sync again
	 */
	broadcast_sink.requested_bis_sync = 0U;

	if (!atomic_test_bit(flags, FLAG_BROADCAST_SYNCED) &&
	    !atomic_test_bit(flags, FLAG_BROADCAST_SYNCING)) {
		return;
	}

	printk("Stopping the BIS reception\n");

	/* The BIS streams are stopped as part of this, and the Broadcast Receive State then shows
	 * the BISes as not synced. The PA sync is kept, so the BIS sync can quickly be requested
	 * again.
	 */
	err = bt_bap_broadcast_sink_stop(broadcast_sink.bap_broadcast_sink);
	if (err != 0) {
		/* If the BIG still gets synced, broadcast_stream_started_cb() stops it */
		printk("Failed to stop Broadcast Sink: %d\n", err);
	}
}

int bap_broadcast_sink_init(void)
{
	static struct bt_bap_scan_delegator_cb scan_delegator_cbs = {
		.recv_state_updated = recv_state_updated_cb,
		.pa_sync_req = pa_sync_req_cb,
		.pa_sync_term_req = pa_sync_term_req_cb,
		.broadcast_code = broadcast_code_cb,
		.bis_sync_req = bis_sync_req_cb,
		.remove_source = remove_source_cb,
	};
	static struct bt_bap_broadcast_sink_cb broadcast_sink_cbs = {
		.base_recv = base_recv_cb,
		.syncable = syncable_cb,
		.started = sink_started_cb,
		.stopped = sink_stopped_cb,
	};
	static struct bt_bap_stream_ops broadcast_stream_ops = {
		.started = broadcast_stream_started_cb,
		.stopped = broadcast_stream_stopped_cb,
		.recv = broadcast_stream_recv_cb,
	};
	static struct bt_le_per_adv_sync_cb bap_pa_sync_cb = {
		.synced = bap_pa_sync_synced_cb,
		.term = bap_pa_sync_terminated_cb,
	};
	static struct bt_le_scan_cb bap_scan_cb = {
		.recv = broadcast_scan_recv,
	};
	int err;

	err = bt_bap_scan_delegator_register(&scan_delegator_cbs);
	if (err != 0) {
		printk("Scan delegator register failed (err %d)\n", err);

		return err;
	}

	err = bt_bap_broadcast_sink_register_cb(&broadcast_sink_cbs);
	if (err != 0) {
		printk("Failed to register BAP broadcast sink callbacks (err %d)\n", err);

		return err;
	}

	ARRAY_FOR_EACH_PTR(broadcast_streams, broadcast_stream) {
		bt_bap_stream_cb_register(&broadcast_stream->stream, &broadcast_stream_ops);
	}

	bt_le_per_adv_sync_cb_register(&bap_pa_sync_cb);

	if (IS_ENABLED(CONFIG_NRF54L15_AUDIO_HEADSET_SCAN_SELF)) {
		bt_le_scan_cb_register(&bap_scan_cb);

		err = check_start_scan();
		if (err != 0) {
			return err;
		}
	}

	return 0;
}
