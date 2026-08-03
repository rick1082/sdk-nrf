/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Air mouse with an LE Audio microphone for the Seeed XIAO nRF54L15 Sense.
 *
 * The on-board LSM6DS3TR-C gyroscope is sampled at 100 Hz; yaw and pitch rates
 * are integrated into relative cursor deltas and pushed to the host as HID over
 * GATT (HOGP) mouse reports. The USR button is the left mouse button.
 *
 * The same connection also carries a BAP unicast server that streams the
 * on-board PDM microphone as LC3 audio; see le_audio_mic.c.
 */

#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/input/input.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include <bluetooth/services/hids.h>

#include "le_audio_mic.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(air_mouse, LOG_LEVEL_INF);

#define IMU_NODE DT_ALIAS(imu0)

BUILD_ASSERT(DT_NODE_HAS_STATUS(IMU_NODE, okay),
	     "imu0 alias is not enabled - this build target has no IMU");

/* WHO_AM_I sits at the same address across the LSM6DS family */
#define IMU_REG_WHO_AM_I 0x0f

/* ------------------------------------------------------------------------- */
/* Motion tuning                                                             */
/* ------------------------------------------------------------------------- */

/* Gyro sampling period. Matches the 104 Hz ODR configured in prj.conf. */
#define SAMPLE_PERIOD_MS 10
/* One HID report per two samples: 50 Hz is comfortably below the negotiated
 * connection interval, so reports never pile up in the controller.
 */
#define SAMPLES_PER_REPORT 2

/* Cursor pixels per radian of rotation. Raise for a twitchier pointer. */
#define CURSOR_GAIN_PX_PER_RAD 900.0f
/* Rates below this are treated as zero, so gyro bias does not creep. */
#define GYRO_DEADZONE_RAD_S 0.04f
/* HID relative axes are 8-bit signed. */
#define CURSOR_DELTA_MAX 127

/*
 * Axis mapping, with the board held flat and the USB connector pointing away
 * from you: yaw (about Z) sweeps the cursor horizontally, pitch (about X)
 * sweeps it vertically. Flip a sign if your pointer runs the wrong way.
 */
#define GYRO_IDX_CURSOR_X 2
#define GYRO_IDX_CURSOR_Y 0
#define CURSOR_X_SIGN (-1.0f)
#define CURSOR_Y_SIGN (-1.0f)

/* ------------------------------------------------------------------------- */
/* HID                                                                       */
/* ------------------------------------------------------------------------- */

#define BASE_USB_HID_SPEC_VERSION 0x0101

/* Report layout: buttons, X, Y, wheel. */
#define INPUT_REP_MOUSE_LEN   4
#define INPUT_REP_MOUSE_INDEX 0
#define INPUT_REP_MOUSE_ID    1

#define MOUSE_BTN_LEFT BIT(0)

BT_HIDS_DEF(hids_obj, INPUT_REP_MOUSE_LEN);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
		      (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
		      (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
					  BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),
					  BT_UUID_16_ENCODE(BT_UUID_MICS_VAL)),
	BT_DATA_BYTES(BT_DATA_SVC_DATA16, LE_AUDIO_MIC_ADV_SVC_DATA),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static struct bt_conn *cur_conn;
static bool in_boot_mode;

/* Set from the input callback, consumed by the motion loop. */
static atomic_t button_state;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static void advertising_start(void)
{
	int err;

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err == -EALREADY) {
		return;
	}
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising as \"%s\"", CONFIG_BT_DEVICE_NAME);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Failed to connect to %s (0x%02x)", addr, err);
		advertising_start();
		return;
	}

	LOG_INF("Connected to %s", addr);

	if (bt_hids_connected(&hids_obj, conn)) {
		LOG_ERR("Failed to notify HID service about connection");
		return;
	}

	cur_conn = bt_conn_ref(conn);
	in_boot_mode = false;
	gpio_pin_set_dt(&led, 1);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02x)", reason);

	if (bt_hids_disconnected(&hids_obj, conn)) {
		LOG_ERR("Failed to notify HID service about disconnection");
	}

	if (cur_conn == conn) {
		bt_conn_unref(cur_conn);
		cur_conn = NULL;
	}

	gpio_pin_set_dt(&led, 0);
	advertising_start();
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	if (err) {
		LOG_ERR("Security failed: level %u err %d", level, err);
	} else {
		LOG_INF("Security level %u - ready to move the cursor", level);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	LOG_INF("Pairing complete (bonded: %s)", bonded ? "yes" : "no");
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	LOG_WRN("Pairing failed (reason %d)", reason);
}

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

static void hids_pm_evt_handler(enum bt_hids_pm_evt evt, struct bt_conn *conn)
{
	switch (evt) {
	case BT_HIDS_PM_EVT_BOOT_MODE_ENTERED:
		LOG_INF("Boot protocol mode entered");
		in_boot_mode = true;
		break;
	case BT_HIDS_PM_EVT_REPORT_MODE_ENTERED:
		LOG_INF("Report protocol mode entered");
		in_boot_mode = false;
		break;
	default:
		break;
	}
}

static int hid_init(void)
{
	struct bt_hids_init_param hids_init_param = {0};
	struct bt_hids_inp_rep *hids_inp_rep;

	static const uint8_t report_map[] = {
		0x05, 0x01,       /* Usage Page (Generic Desktop) */
		0x09, 0x02,       /* Usage (Mouse) */
		0xA1, 0x01,       /* Collection (Application) */
		0x85, INPUT_REP_MOUSE_ID, /* Report Id */
		0x09, 0x01,       /* Usage (Pointer) */
		0xA1, 0x00,       /* Collection (Physical) */

		0x05, 0x09,       /* Usage Page (Button) */
		0x19, 0x01,       /* Usage Minimum (1) */
		0x29, 0x03,       /* Usage Maximum (3) */
		0x15, 0x00,       /* Logical Minimum (0) */
		0x25, 0x01,       /* Logical Maximum (1) */
		0x75, 0x01,       /* Report Size (1) */
		0x95, 0x03,       /* Report Count (3) */
		0x81, 0x02,       /* Input (Data, Variable, Absolute) */
		0x75, 0x05,       /* Report Size (5) */
		0x95, 0x01,       /* Report Count (1) */
		0x81, 0x01,       /* Input (Constant) for padding */

		0x05, 0x01,       /* Usage Page (Generic Desktop) */
		0x09, 0x30,       /* Usage (X) */
		0x09, 0x31,       /* Usage (Y) */
		0x09, 0x38,       /* Usage (Wheel) */
		0x15, 0x81,       /* Logical Minimum (-127) */
		0x25, 0x7F,       /* Logical Maximum (127) */
		0x75, 0x08,       /* Report Size (8) */
		0x95, 0x03,       /* Report Count (3) */
		0x81, 0x06,       /* Input (Data, Variable, Relative) */

		0xC0,             /* End Collection (Physical) */
		0xC0,             /* End Collection (Application) */
	};

	hids_init_param.rep_map.data = report_map;
	hids_init_param.rep_map.size = sizeof(report_map);

	hids_init_param.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
	hids_init_param.info.b_country_code = 0x00;
	hids_init_param.info.flags = (BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE);

	hids_inp_rep = &hids_init_param.inp_rep_group_init.reports[INPUT_REP_MOUSE_INDEX];
	hids_inp_rep->size = INPUT_REP_MOUSE_LEN;
	hids_inp_rep->id = INPUT_REP_MOUSE_ID;
	hids_init_param.inp_rep_group_init.cnt++;

	hids_init_param.is_mouse = true;
	hids_init_param.pm_evt_handler = hids_pm_evt_handler;

	return bt_hids_init(&hids_obj, &hids_init_param);
}

static int mouse_report_send(uint8_t buttons, int8_t x, int8_t y)
{
	uint8_t report[INPUT_REP_MOUSE_LEN];

	if (!cur_conn) {
		return -ENOTCONN;
	}

	if (in_boot_mode) {
		return bt_hids_boot_mouse_inp_rep_send(&hids_obj, cur_conn, &buttons, x, y, NULL);
	}

	report[0] = buttons;
	report[1] = (uint8_t)x;
	report[2] = (uint8_t)y;
	report[3] = 0; /* wheel */

	return bt_hids_inp_rep_send(&hids_obj, cur_conn, INPUT_REP_MOUSE_INDEX,
				    report, sizeof(report), NULL);
}

/* ------------------------------------------------------------------------- */
/* Input                                                                     */
/* ------------------------------------------------------------------------- */

static void button_cb(struct input_event *evt, void *user_data)
{
	if (evt->type != INPUT_EV_KEY || evt->code != INPUT_KEY_ENTER) {
		return;
	}

	if (evt->value) {
		atomic_or(&button_state, MOUSE_BTN_LEFT);
	} else {
		atomic_and(&button_state, ~MOUSE_BTN_LEFT);
	}
}

INPUT_CALLBACK_DEFINE(NULL, button_cb, NULL);

/* ------------------------------------------------------------------------- */
/* IMU                                                                       */
/* ------------------------------------------------------------------------- */

/*
 * The IMU and PDM share a power rail switched by P0.01 (pdm_imu_pwr). A soft
 * reset leaves P0 - which lives in the always-on domain - untouched, so the
 * sensor stays powered and probes fine; a pin reset or a power cycle drops the
 * rail and the LSM6DS3TR-C needs ~15 ms to boot before it answers on I2C.
 *
 * regulator-fixed cannot cover that gap on its own: with regulator-boot-on set,
 * regulator_common_init() takes the "already enabled" path and never applies
 * startup-delay-us. Hold the boot sequence here instead, after the rail is
 * switched on (priority 41/45) and before the sensor is probed (priority 90).
 */
#define IMU_POWER_ON_DELAY_MS 50
/* SYS_INIT needs a literal priority, so assert the ordering instead. */
#define IMU_POWER_SETTLE_INIT_PRIORITY 89

BUILD_ASSERT(IMU_POWER_SETTLE_INIT_PRIORITY < CONFIG_SENSOR_INIT_PRIORITY,
	     "The IMU rail must settle before the sensor driver probes it");

static int imu_power_settle(void)
{
	const struct device *const bus = DEVICE_DT_GET(DT_BUS(IMU_NODE));

	k_msleep(IMU_POWER_ON_DELAY_MS);

	/*
	 * A reset in the middle of an I2C transfer leaves the sensor holding SDA
	 * low, and every later transfer - including the driver's WHO_AM_I probe -
	 * then fails until the bus is clocked out. Recover before probing.
	 */
	if (device_is_ready(bus)) {
		(void)i2c_recover_bus(bus);
	}

	return 0;
}

SYS_INIT(imu_power_settle, POST_KERNEL, IMU_POWER_SETTLE_INIT_PRIORITY);

static void imu_print_info(const struct device *imu)
{
	const struct i2c_dt_spec bus = I2C_DT_SPEC_GET(IMU_NODE);
	uint8_t who_am_i;
	int err;

	LOG_INF("IMU device    : %s", imu->name);
	LOG_INF("Devicetree    : %s (compatible \"%s\")",
		DT_NODE_FULL_NAME(IMU_NODE), DT_PROP(IMU_NODE, compatible_IDX_0));
	LOG_INF("Bus           : %s, address 0x%02x, %d Hz",
	       bus.bus->name, bus.addr, DT_PROP(DT_BUS(IMU_NODE), clock_frequency));

	err = i2c_reg_read_byte_dt(&bus, IMU_REG_WHO_AM_I, &who_am_i);
	if (err) {
		LOG_ERR("WHO_AM_I      : read failed (%d)", err);
	} else {
		LOG_INF("WHO_AM_I      : 0x%02x", who_am_i);
	}

	LOG_INF("Accelerometer : +/-%dg, ODR setting %d",
		CONFIG_LSM6DSL_ACCEL_FS, CONFIG_LSM6DSL_ACCEL_ODR);
	LOG_INF("Gyroscope     : +/-%ddps, ODR setting %d",
		CONFIG_LSM6DSL_GYRO_FS, CONFIG_LSM6DSL_GYRO_ODR);
}

/* Returns the yaw/pitch rates in rad/s, deadzone already applied. */
static int imu_read_rates(const struct device *imu, float *rate_x, float *rate_y)
{
	struct sensor_value gyro[3];
	float rates[2];
	int err;

	err = sensor_sample_fetch(imu);
	if (err) {
		return err;
	}

	err = sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (err) {
		return err;
	}

	rates[0] = CURSOR_X_SIGN * (float)sensor_value_to_double(&gyro[GYRO_IDX_CURSOR_X]);
	rates[1] = CURSOR_Y_SIGN * (float)sensor_value_to_double(&gyro[GYRO_IDX_CURSOR_Y]);

	for (int i = 0; i < 2; i++) {
		if ((rates[i] < GYRO_DEADZONE_RAD_S) && (rates[i] > -GYRO_DEADZONE_RAD_S)) {
			rates[i] = 0.0f;
		}
	}

	*rate_x = rates[0];
	*rate_y = rates[1];

	return 0;
}

/*
 * Pops the whole-pixel part of an accumulator, clamped to the HID range, and
 * leaves the remainder behind so slow drags are not rounded away.
 */
static int8_t accum_take(float *accum)
{
	float whole = (*accum < 0.0f) ? ceilf(*accum) : floorf(*accum);
	int delta = CLAMP((int)whole, -CURSOR_DELTA_MAX, CURSOR_DELTA_MAX);

	*accum -= (float)delta;

	return (int8_t)delta;
}

int main(void)
{
	const struct device *const imu = DEVICE_DT_GET(IMU_NODE);
	const float dt = SAMPLE_PERIOD_MS / 1000.0f;
	bool imu_ok;
	float x_accum = 0.0f;
	float y_accum = 0.0f;
	uint8_t last_buttons = 0;
	unsigned int tick = 0;
	int err;

	LOG_INF("XIAO nRF54L15 Sense air mouse + LE Audio mic - %s", CONFIG_BOARD_TARGET);

	/* A dead IMU costs the pointer, but the microphone, MICS and HID still
	 * work, so carry on rather than bringing the whole device down.
	 */
	imu_ok = device_is_ready(imu);
	if (!imu_ok) {
		LOG_ERR("IMU %s is not ready - continuing without pointer motion", imu->name);
	}

	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("LED is not ready");
		return 0;
	}
	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

	if (imu_ok) {
		imu_print_info(imu);
	}

	err = hid_init();
	if (err) {
		LOG_ERR("HID service init failed (err %d)", err);
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		LOG_ERR("Failed to register auth info callbacks (err %d)", err);
		return 0;
	}

	/* Starts the HF clock and the PDM peripheral; must precede bt_enable(). */
	err = le_audio_mic_init();
	if (err) {
		LOG_ERR("LE Audio microphone init failed (err %d)", err);
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	/* ASCS is a dynamic GATT service, and with CONFIG_BT_SETTINGS those
	 * cannot be registered until the settings have been loaded.
	 */
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = le_audio_mic_start();
	if (err) {
		LOG_ERR("LE Audio microphone start failed (err %d)", err);
		return 0;
	}

	advertising_start();

	while (1) {
		float rate_x;
		float rate_y;
		uint8_t buttons;

		k_sleep(K_MSEC(SAMPLE_PERIOD_MS));

		if (!imu_ok) {
			continue;
		}

		if (imu_read_rates(imu, &rate_x, &rate_y)) {
			continue;
		}

		x_accum += rate_x * dt * CURSOR_GAIN_PX_PER_RAD;
		y_accum += rate_y * dt * CURSOR_GAIN_PX_PER_RAD;

		if ((++tick % SAMPLES_PER_REPORT) != 0) {
			continue;
		}

		buttons = (uint8_t)atomic_get(&button_state);

		if (!cur_conn) {
			/* Nothing to report to; do not let the deltas run away. */
			x_accum = 0.0f;
			y_accum = 0.0f;
			continue;
		}

		int8_t x = accum_take(&x_accum);
		int8_t y = accum_take(&y_accum);

		if (!x && !y && (buttons == last_buttons)) {
			continue;
		}

		err = mouse_report_send(buttons, x, y);
		if (err) {
			/* Report was dropped - put the motion back for next time. */
			x_accum += (float)x;
			y_accum += (float)y;
			if (err != -ENOMEM && err != -EACCES) {
				LOG_WRN("Report send failed (err %d)", err);
			}
			continue;
		}

		last_buttons = buttons;
	}

	return 0;
}
