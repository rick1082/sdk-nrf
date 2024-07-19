/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "hw_codec.h"

#include <zephyr/kernel.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#include "macros_common.h"
#include "zbus_common.h"
#include "cs47l63.h"
#include "cs47l63_spec.h"
#include "cs47l63_reg_conf.h"
#include "cs47l63_comm.h"

#define I2C1_NODE DT_NODELABEL(mysensor)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C1_NODE);

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hw_codec, CONFIG_MODULE_HW_CODEC_LOG_LEVEL);

#define VOLUME_ADJUST_STEP_DB 3
#define BASE_10		      10

ZBUS_SUBSCRIBER_DEFINE(volume_evt_sub, CONFIG_VOLUME_MSG_SUB_QUEUE_SIZE);

static k_tid_t volume_msg_sub_thread_id;
static struct k_thread volume_msg_sub_thread_data;

K_THREAD_STACK_DEFINE(volume_msg_sub_thread_stack, CONFIG_VOLUME_MSG_SUB_STACK_SIZE);

/**
 * @brief	Convert the zbus volume to the actual volume setting for the HW codec.
 *
 * @note	The range for zbus volume is from 0 to 255 and the
 *		range for HW codec volume is from 0 to 128.
 */
static uint16_t zbus_vol_conversion(uint8_t volume)
{
	return (((uint16_t)volume + 1) / 2);
}

/**
 * @brief	Handle volume events from zbus.
 */
static void volume_msg_sub_thread(void)
{
	int ret;

	const struct zbus_channel *chan;

	while (1) {
		ret = zbus_sub_wait(&volume_evt_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		struct volume_msg msg;

		ret = zbus_chan_read(chan, &msg, ZBUS_READ_TIMEOUT_MS);
		if (ret) {
			LOG_ERR("Failed to read from zbus: %d", ret);
		}

		uint8_t event = msg.event;
		uint8_t volume = msg.volume;

		LOG_DBG("Received event = %d, volume = %d", event, volume);

		switch (event) {
		case VOLUME_UP:
			LOG_DBG("Volume up received");
			ret = hw_codec_volume_increase();
			if (ret) {
				LOG_ERR("Failed to increase volume, ret: %d", ret);
			}
			break;
		case VOLUME_DOWN:
			LOG_DBG("Volume down received");
			ret = hw_codec_volume_decrease();
			if (ret) {
				LOG_ERR("Failed to decrease volume, ret: %d", ret);
			}
			break;
		case VOLUME_SET:
			LOG_DBG("Volume set received");
			ret = hw_codec_volume_set(zbus_vol_conversion(volume));
			if (ret) {
				LOG_ERR("Failed to set the volume to %d, ret: %d", volume, ret);
			}
			break;
		case VOLUME_MUTE:
			LOG_DBG("Volume mute received");
			ret = hw_codec_volume_mute();
			if (ret) {
				LOG_ERR("Failed to mute volume, ret: %d", ret);
			}
			break;
		case VOLUME_UNMUTE:
			LOG_DBG("Volume unmute received");
			ret = hw_codec_volume_unmute();
			if (ret) {
				LOG_ERR("Failed to unmute volume, ret: %d", ret);
			}
			break;
		default:
			LOG_WRN("Unexpected/unhandled volume event: %d", event);
			break;
		}

		STACK_USAGE_PRINT("volume_msg_thread", &volume_msg_sub_thread_data);
	}
}

int hw_codec_volume_set(uint8_t set_val)
{
	return 0;
}

int hw_codec_volume_adjust(int8_t adjustment_db)
{
	return 0;
}

int hw_codec_volume_decrease(void)
{
	return 0;
}

int hw_codec_volume_increase(void)
{
	return 0;
}

int hw_codec_volume_mute(void)
{
	return 0;
}

int hw_codec_volume_unmute(void)
{
	return 0;
}

int hw_codec_default_conf_enable(void)
{
	return 0;
}
uint8_t wm8960_reg[19][2] = {
	{0x1e, 0x00}, {0x32, 0xc0}, {0x35, 0xe0}, {0x5e, 0x0c}, {0x45, 0x00},
	{0x4b, 0x00}, {0x05, 0x79}, {0x07, 0x79}, {0x0a, 0x00}, {0x68, 0x28},
	{0x6a, 0x00}, {0x6c, 0x00}, {0x6e, 0x00}, {0x35, 0xE1}, {0x08, 0x05},
	{0x0e, 0x02}, {0x32, 0xc0}, {0x35, 0xE1}, {0x5e, 0x0c},
};

int hw_codec_soft_reset(void)
{

	return 0;
}

int hw_codec_init(void)
{
	int ret;

	for (int i = 0; i < 19; i++){
		ret = i2c_write_dt(&dev_i2c, wm8960_reg[i], 2);
		if (ret != 0) {
			LOG_WRN("Failed to write/read I2C device address %X", dev_i2c.addr);
		} else {
			LOG_WRN("I2S write properly");
		}
	}

	volume_msg_sub_thread_id = k_thread_create(
		&volume_msg_sub_thread_data, volume_msg_sub_thread_stack,
		CONFIG_VOLUME_MSG_SUB_STACK_SIZE, (k_thread_entry_t)volume_msg_sub_thread, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_VOLUME_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(volume_msg_sub_thread_id, "VOLUME_MSG_SUB");
	ERR_CHK(ret);

	return 0;
}
