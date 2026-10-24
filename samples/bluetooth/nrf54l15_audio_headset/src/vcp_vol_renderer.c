/** @file
 *  @brief Bluetooth Volume Control Profile (VCP) Volume Renderer role.
 *
 *  Copyright (c) 2020 Bose Corporation
 *  Copyright (c) 2020-2024 Nordic Semiconductor ASA
 *  Copyright (c) 2022 Codecoup
 *  Copyright 2023 NXP
 *
 *  SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/audio/vcp.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <dk_buttons_and_leds.h>

#include "nrf54l15_audio_headset.h"

/* The 50 dB DAC volume range is mapped to the volume range 0-255, so a step of 13 changes the
 * volume by about 2.5 dB
 */
#define VOLUME_STEP 13U

static struct bt_vcp_included vcp_included;

static void vcs_state_cb(struct bt_conn *conn, int err, uint8_t volume, uint8_t mute)
{
	if (err) {
		printk("VCS state get failed (%d)\n", err);
	} else {
		printk("VCS volume %u, mute %u\n", volume, mute);
		audio_out_volume_set(volume, mute == BT_VCP_STATE_MUTED);
	}
}

static void vcs_flags_cb(struct bt_conn *conn, int err, uint8_t flags)
{
	if (err) {
		printk("VCS flags get failed (%d)\n", err);
	} else {
		printk("VCS flags 0x%02X\n", flags);
	}
}

static struct bt_vcp_vol_rend_cb vcp_cbs = {
	.state = vcs_state_cb,
	.flags = vcs_flags_cb,
};

/* The volume is changed through the Volume Control Service, so that the phone is notified and
 * the output follows in vcs_state_cb()
 */
static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	const uint32_t pressed = button_state & has_changed;
	int err;

	if ((pressed & DK_BTN1_MSK) != 0U) {
		/* Turning the volume up also unmutes, like on a phone */
		err = bt_vcp_vol_rend_unmute_vol_up();
		if (err != 0) {
			printk("Failed to turn the volume up (err %d)\n", err);
		}
	}

	if ((pressed & DK_BTN2_MSK) != 0U) {
		err = bt_vcp_vol_rend_vol_down();
		if (err != 0) {
			printk("Failed to turn the volume down (err %d)\n", err);
		}
	}
}

int vcp_vol_renderer_init(void)
{
	int err;
	struct bt_vcp_vol_rend_register_param vcp_register_param;

	memset(&vcp_register_param, 0, sizeof(vcp_register_param));

	vcp_register_param.step = VOLUME_STEP;
	vcp_register_param.mute = BT_VCP_STATE_UNMUTED;
	vcp_register_param.volume = 100;
	vcp_register_param.cb = &vcp_cbs;

	err = bt_vcp_vol_rend_register(&vcp_register_param);
	if (err) {
		return err;
	}

	err = bt_vcp_vol_rend_included_get(&vcp_included);
	if (err != 0) {
		return err;
	}

	audio_out_volume_set(vcp_register_param.volume,
			     vcp_register_param.mute == BT_VCP_STATE_MUTED);

	err = dk_buttons_init(button_handler);
	if (err != 0) {
		printk("Failed to initialize the buttons (err %d)\n", err);
		return err;
	}

	return 0;
}
