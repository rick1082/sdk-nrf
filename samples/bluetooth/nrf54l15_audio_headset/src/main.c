/*
 * Copyright 2023 NXP
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>

#include <zephyr/autoconf.h>
#include <zephyr/bluetooth/assigned_numbers.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/lc3.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/bluetooth/audio/tmap.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#include "nrf54l15_audio_headset.h"

#define TMAP_ROLE (BT_TMAP_ROLE_CT | BT_TMAP_ROLE_UMR | BT_TMAP_ROLE_BMR)

/* Cover the LC3 presets used by phones for both media (e.g. 48_2, 48_4) and calls (16_2, 32_2) */
#define SUPPORTED_FREQ                                                                             \
	(BT_AUDIO_CODEC_CAP_FREQ_16KHZ | BT_AUDIO_CODEC_CAP_FREQ_24KHZ |                           \
	 BT_AUDIO_CODEC_CAP_FREQ_32KHZ | BT_AUDIO_CODEC_CAP_FREQ_48KHZ)
#define SUPPORTED_DURATION (BT_AUDIO_CODEC_CAP_DURATION_7_5 | BT_AUDIO_CODEC_CAP_DURATION_10)
#define MIN_OCTETS_PER_FRAME 26U
#define MAX_OCTETS_PER_FRAME 155U
#define FRAMES_PER_SDU       1U
/* Stereo output, with one sink ASE per channel */
#define SINK_LOCATION   (BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT)
#define SOURCE_LOCATION BT_AUDIO_LOCATION_FRONT_LEFT

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
		      BT_BYTES_LIST_LE16(BT_APPEARANCE_WEARABLE_AUDIO_DEVICE_EARBUD)),
	BT_DATA_BYTES(BT_DATA_UUID16_SOME, BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_CAS_VAL), BT_UUID_16_ENCODE(BT_UUID_TMAS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BASS_VAL)),
	BT_DATA_BYTES(BT_DATA_SVC_DATA16, BT_UUID_16_ENCODE(BT_UUID_TMAS_VAL),
		      BT_BYTES_LIST_LE16(TMAP_ROLE)),
	BT_DATA_BYTES(BT_DATA_SVC_DATA16, BT_UUID_16_ENCODE(BT_UUID_CAS_VAL),
		      BT_AUDIO_UNICAST_ANNOUNCEMENT_TARGETED),
	BT_DATA_BYTES(BT_DATA_SVC_DATA16, BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),
		      BT_AUDIO_UNICAST_ANNOUNCEMENT_TARGETED, BT_BYTES_LIST_LE16(SINK_CONTEXT),
		      BT_BYTES_LIST_LE16(SOURCE_CONTEXT), 0x00 /* Metadata length */),
	BT_DATA_BYTES(BT_DATA_SVC_DATA16, BT_UUID_16_ENCODE(BT_UUID_BASS_VAL)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static struct bt_conn *default_conn;

static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_disconnected, 0, 1);

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0) {
		printk("Failed to connect to %s %u %s\n", bt_conn_dst_str(conn),
		       err, bt_hci_err_to_str(err));

		return;
	}

	printk("Connected: %s\n", bt_conn_dst_str(conn));
	default_conn = bt_conn_ref(conn);
	k_sem_give(&sem_connected);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (conn != default_conn) {
		return;
	}

	printk("Disconnected: %s, reason 0x%02x %s\n", bt_conn_dst_str(conn),
	       reason, bt_hci_err_to_str(reason));

	bt_conn_unref(default_conn);
	default_conn = NULL;

	k_sem_give(&sem_disconnected);
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	if (err == 0) {
		printk("Security changed: level %u\n", level);
	} else {
		printk("Failed to set security level: %s(%u)\n", bt_security_err_to_str(err), err);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

/** Register the PAC record, location and contexts for a direction */
static int register_pac(enum bt_audio_dir dir, enum bt_audio_context context,
			struct bt_pacs_cap *cap, enum bt_audio_location location)
{
	int err;

	err = bt_pacs_cap_register(dir, cap);
	if (err != 0) {
		printk("Failed to register capabilities (err %d)\n", err);
		return err;
	}

	err = bt_pacs_set_location(dir, location);
	if (err != 0) {
		printk("Failed to set location (err %d)\n", err);
		return err;
	}

	err = bt_pacs_set_supported_contexts(dir, context);
	if (err != 0 && err != -EALREADY) {
		printk("Failed to set supported contexts (err %d)\n", err);
		return err;
	}

	err = bt_pacs_set_available_contexts(dir, context);
	if (err != 0 && err != -EALREADY) {
		printk("Failed to set available contexts (err %d)\n", err);
		return err;
	}

	return 0;
}

/* The sink PAC record is used for both the unicast (CIS) and the broadcast (BIS) sink */
static int pacs_init(void)
{
	static const struct bt_audio_codec_cap sink_codec_cap = BT_AUDIO_CODEC_CAP_LC3(
		SUPPORTED_FREQ, SUPPORTED_DURATION, BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1),
		MIN_OCTETS_PER_FRAME, MAX_OCTETS_PER_FRAME, FRAMES_PER_SDU, SINK_CONTEXT);
	static const struct bt_audio_codec_cap source_codec_cap = BT_AUDIO_CODEC_CAP_LC3(
		SUPPORTED_FREQ, SUPPORTED_DURATION, BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1),
		MIN_OCTETS_PER_FRAME, MAX_OCTETS_PER_FRAME, FRAMES_PER_SDU, SOURCE_CONTEXT);
	static struct bt_pacs_cap sink_cap = {
		.codec_cap = &sink_codec_cap,
	};
	static struct bt_pacs_cap source_cap = {
		.codec_cap = &source_codec_cap,
	};
	const struct bt_pacs_register_param pacs_param = {
		.snk_pac = true,
		.snk_loc = true,
		.src_pac = true,
		.src_loc = true,
	};
	int err;

	err = bt_pacs_register(&pacs_param);
	if (err != 0) {
		printk("Could not register PACS (err %d)\n", err);
		return err;
	}

	err = register_pac(BT_AUDIO_DIR_SINK, SINK_CONTEXT, &sink_cap, SINK_LOCATION);
	if (err != 0) {
		return err;
	}

	return register_pac(BT_AUDIO_DIR_SOURCE, SOURCE_CONTEXT, &source_cap, SOURCE_LOCATION);
}

int main(void)
{
	struct bt_le_ext_adv *adv;
	int err;

	err = audio_out_init();
	if (err != 0) {
		printk("Audio output init failed (err %d)\n", err);
		return err;
	}

	printk("Audio output initialized\n");

	err = bt_enable(NULL);
	if (err != 0) {
		printk("Bluetooth init failed (err %d)\n", err);
		return err;
	}

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		/* Restore bonds so that a phone can reconnect after a reset */
		err = settings_load();
		if (err != 0) {
			printk("Failed to load settings (err %d)\n", err);
			return err;
		}
	}

	printk("Initializing TMAP and setting role\n");
	err = bt_tmap_register(TMAP_ROLE);
	if (err != 0) {
		return err;
	}

	err = vcp_vol_renderer_init();
	if (err != 0) {
		return err;
	}
	printk("VCP initialized\n");

	err = pacs_init();
	if (err != 0) {
		return err;
	}
	printk("PACS initialized\n");

	err = bap_unicast_sr_init();
	if (err != 0) {
		return err;
	}
	printk("BAP Unicast Server initialized\n");

	err = bap_broadcast_sink_init();
	if (err != 0) {
		return err;
	}
	printk("BAP Broadcast Sink initialized\n");

	err = bt_le_ext_adv_create(BT_BAP_ADV_PARAM_CONN_QUICK, NULL, &adv);
	if (err != 0) {
		printk("Failed to create advertising set (err %d)\n", err);
		return err;
	}

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err != 0) {
		printk("Failed to set advertising data (err %d)\n", err);
		return err;
	}

	while (true) {
		err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
		if (err != 0) {
			printk("Failed to start advertising set (err %d)\n", err);
			return err;
		}

		printk("Advertising successfully started\n");

		/* The connected phone acts as CAP Initiator (CIS) and/or Broadcast Assistant (BIS).
		 * All requests are handled in the callbacks; just wait for it to go away again.
		 */
		k_sem_take(&sem_connected, K_FOREVER);
		k_sem_take(&sem_disconnected, K_FOREVER);
	}

	return 0;
}
