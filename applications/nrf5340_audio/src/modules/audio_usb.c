/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdint.h>
#include <stdlib.h>

#include <sample_usbd.h>

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_uac2.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#include <data_fifo.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/class/usbd_hid.h>
#include "macros_common.h"

LOG_MODULE_REGISTER(uac2_sample, LOG_LEVEL_INF);

#define HEADPHONES_OUT_TERMINAL_ID UAC2_ENTITY_ID(DT_NODELABEL(out_terminal))
#define MICROPHONE_IN_TERMINAL_ID  UAC2_ENTITY_ID(DT_NODELABEL(in_terminal))

#define SAMPLES_PER_SOF	   48
#define SAMPLE_FREQUENCY   (SAMPLES_PER_SOF * 1000)
#define SAMPLE_BIT_WIDTH   16
#define NUMBER_OF_CHANNELS 2
#define BYTES_PER_SAMPLE   DIV_ROUND_UP(SAMPLE_BIT_WIDTH, 8)
#define BYTES_PER_SLOT	   (BYTES_PER_SAMPLE * NUMBER_OF_CHANNELS)
#define MIN_BLOCK_SIZE	   ((SAMPLES_PER_SOF - 1) * BYTES_PER_SLOT)
#define BLOCK_SIZE	   (SAMPLES_PER_SOF * BYTES_PER_SLOT)
#define MAX_BLOCK_SIZE	   ((SAMPLES_PER_SOF + 1) * BYTES_PER_SLOT)

static struct data_fifo *fifo_tx;
static struct data_fifo *fifo_rx;

#define USB_FRAME_SIZE_STEREO                                                                      \
	(((CONFIG_AUDIO_SAMPLE_RATE_HZ * CONFIG_AUDIO_BIT_DEPTH_OCTETS) / 8000) * 2)
NET_BUF_POOL_FIXED_DEFINE(pool_out, CONFIG_FIFO_FRAME_SPLIT_NUM, USB_FRAME_SIZE_STEREO, 8,
			  net_buf_destroy);

const struct device *headset = DEVICE_DT_GET(DT_NODELABEL(uac2_headset));
const struct device *hid_dev_keyboard;
const struct device *hid_dev_mouse;
static uint32_t rx_num_overruns;
static bool rx_first_data;
static bool tx_first_data;
/* Absolute minimum is 5 buffers (1 actively consumed by I2S, 2nd queued as next
 * buffer, 3rd acquired by USB stack to receive data to, and 2 to handle SOF/I2S
 * offset errors), but add 2 additional buffers to prevent out of memory errors
 * when USB host decides to perform rapid terminal enable/disable cycles.
 */
#define I2S_BUFFERS_COUNT 7
K_MEM_SLAB_DEFINE_STATIC(i2s_tx_slab, ROUND_UP(MAX_BLOCK_SIZE, UDC_BUF_GRANULARITY),
			 I2S_BUFFERS_COUNT, UDC_BUF_ALIGN);

struct usb_i2s_ctx {
	const struct device *i2s_dev;
	bool headphones_enabled;
	bool microphone_enabled;
	bool i2s_started;
	/* Number of blocks written, used to determine when to start I2S.
	 * Overflows are not a problem becuse this variable is not necessary
	 * after I2S is started.
	 */
	uint8_t i2s_blocks_written;
	struct feedback_ctx *fb;
};

static void uac2_terminal_update_cb(const struct device *dev, uint8_t terminal, bool enabled,
				    bool microframes, void *user_data)
{
	struct usb_i2s_ctx *ctx = user_data;

	/* This sample has only one terminal therefore the callback can simply
	 * ignore the terminal variable.
	 */

	if (terminal == HEADPHONES_OUT_TERMINAL_ID) {
		ctx->headphones_enabled = enabled;
	} else if (terminal == MICROPHONE_IN_TERMINAL_ID) {
		ctx->microphone_enabled = enabled;
	}
}

static void *uac2_get_recv_buf(const struct device *dev, uint8_t terminal, uint16_t size,
			       void *user_data)
{
	ARG_UNUSED(dev);
	struct usb_i2s_ctx *ctx = user_data;
	void *buf = NULL;
	int ret;

	if (terminal == HEADPHONES_OUT_TERMINAL_ID) {
		__ASSERT_NO_MSG(size <= MAX_BLOCK_SIZE);
		if (!ctx->headphones_enabled) {
			LOG_ERR("Buffer request on disabled terminal");
			return NULL;
		}

		ret = k_mem_slab_alloc(&i2s_tx_slab, &buf, K_NO_WAIT);
		if (ret != 0) {
			LOG_INF("ret = %d", ret);
			buf = NULL;
		}
	}
	return buf;
}

static void uac2_data_recv_cb(const struct device *dev, uint8_t terminal, void *buf, uint16_t size,
			      void *user_data)
{
	struct usb_i2s_ctx *ctx = user_data;
	int ret;
	void *data_in;

	if (!ctx->headphones_enabled && !ctx->microphone_enabled) {
		k_mem_slab_free(&i2s_tx_slab, buf);
		return;
	}

	if (fifo_rx == NULL) {
		/* Throwing away data */
		k_mem_slab_free(&i2s_tx_slab, buf);
		return;
	}

	if (!size) {
		/* Zero fill to keep I2S going. If this is transient error, then
		 * this is probably best we can do. Otherwise, host will likely
		 * either disable terminal (or the cable will be disconnected)
		 * which will stop I2S.
		 */
		size = BLOCK_SIZE;
		memset(buf, 0, size);
		sys_cache_data_flush_range(buf, size);
	}

	for (int i = 0; i < 2; i++) {
		ret = data_fifo_pointer_first_vacant_get(fifo_rx, &data_in, K_NO_WAIT);
		if (ret == -ENOMEM) {
			void *temp;
			size_t temp_size;

			rx_num_overruns++;
			if ((rx_num_overruns % 100) == 1) {
				LOG_WRN("USB RX overrun. Num: %d", rx_num_overruns);
			}

			ret = data_fifo_pointer_last_filled_get(fifo_rx, &temp, &temp_size, K_NO_WAIT);
			ERR_CHK(ret);
			data_fifo_block_free(fifo_rx, temp);

			ret = data_fifo_pointer_first_vacant_get(fifo_rx, &data_in, K_NO_WAIT);
		}
		ERR_CHK_MSG(ret, "RX failed to get block");

		memcpy(data_in, buf+(size*i/2), size/2);
		ret = data_fifo_block_lock(fifo_rx, &data_in, size/2);
		ERR_CHK_MSG(ret, "Failed to lock block");
	}

	if (!rx_first_data) {
		LOG_INF("USB RX first data received.");
		rx_first_data = true;
	}

	k_mem_slab_free(&i2s_tx_slab, buf);

}

static void uac2_buf_release_cb(const struct device *dev, uint8_t terminal, void *buf,
				void *user_data)
{
	/* This sample does not send audio data so this won't be called */
}

static volatile bool use_hardcoded_feedback;
static volatile uint32_t hardcoded_feedback = (48 << 14) + 1;

static uint32_t uac2_feedback_cb(const struct device *dev, uint8_t terminal, void *user_data)
{
	/* Sample has only one UAC2 instance with one terminal so both can be
	 * ignored here.
	 */
	ARG_UNUSED(dev);
	ARG_UNUSED(terminal);

	if (use_hardcoded_feedback) {
		return hardcoded_feedback;
	} else {
		return SAMPLES_PER_SOF << 10;
	}
}

#include "pcm_stream_channel_modifier.h"
static uint32_t tx_num_underruns;
static uint8_t __aligned(UDC_BUF_ALIGN) data_buffer[ROUND_UP(96, UDC_BUF_GRANULARITY)] = {0};
static void uac2_sof(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	int ret;
	void *data_out;
	size_t data_out_size;
	uint8_t frame_data[192] = {0};

	if (fifo_tx == NULL) {
		// LOG_INF("returning");
		return;
	}

	for (int i = 0; i < 2; i++) {
		ret = data_fifo_pointer_last_filled_get(fifo_tx, &data_out, &data_out_size,
							K_NO_WAIT);
		if (ret) {
			tx_num_underruns++;
			if ((tx_num_underruns % 100) == 1) {
				// LOG_WRN("USB TX underrun. Num: %d", tx_num_underruns);
			}

			return;
		}
		memcpy(frame_data + (data_out_size * i), data_out, data_out_size);
		data_fifo_block_free(fifo_tx, data_out);
	}
	data_out_size *= 2;

	pscm_one_channel_split(frame_data, data_out_size, 0, 16, data_buffer, &data_out_size);

	if (usbd_uac2_send(dev, MICROPHONE_IN_TERMINAL_ID, data_buffer, 96) < 0) {
		//printk("Failed to send data to USB\n");
	}
}

static struct uac2_ops usb_audio_ops = {
	.sof_cb = uac2_sof,
	.terminal_update_cb = uac2_terminal_update_cb,
	.get_recv_buf = uac2_get_recv_buf,
	.data_recv_cb = uac2_data_recv_cb,
	.buf_release_cb = uac2_buf_release_cb,
	.feedback_cb = uac2_feedback_cb,
};

static struct usb_i2s_ctx main_ctx;

int audio_usb_start(struct data_fifo *fifo_tx_in, struct data_fifo *fifo_rx_in)
{
	if (fifo_rx_in == NULL) {
		return -EINVAL;
	}

	fifo_tx = fifo_tx_in;
	fifo_rx = fifo_rx_in;

	return 0;
}

void audio_usb_stop(void)
{
	LOG_WRN("USB audio stop");
	rx_first_data = false;
	tx_first_data = false;
	fifo_tx = NULL;
	fifo_rx = NULL;
}

int audio_usb_disable(void)
{
	return 0;
}

int audio_usb_send_key(uint8_t key)
{
	return 0;
}

#include <sample_usbd.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>
static const uint8_t hid_report_desc_keyboard[] = HID_KEYBOARD_REPORT_DESC();

enum kb_leds_idx {
	KB_LED_NUMLOCK = 0,
	KB_LED_CAPSLOCK,
	KB_LED_SCROLLLOCK,
	KB_LED_COUNT,
};

static const struct gpio_dt_spec kb_leds[KB_LED_COUNT] = {
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0}),
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0}),
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(led2), gpios, {0}),
};

enum kb_report_idx {
	KB_MOD_KEY = 0,
	KB_RESERVED,
	KB_KEY_CODE1,
	KB_KEY_CODE2,
	KB_KEY_CODE3,
	KB_KEY_CODE4,
	KB_KEY_CODE5,
	KB_KEY_CODE6,
	KB_REPORT_COUNT,
};

struct kb_event {
	uint16_t code;
	int32_t value;
};

K_MSGQ_DEFINE(kb_msgq, sizeof(struct kb_event), 2, 1);


static uint32_t kb_duration;
static bool kb_ready;

static void kb_iface_ready(const struct device *dev, const bool ready)
{
	LOG_INF("HID device %s interface is %s",
		dev->name, ready ? "ready" : "not ready");
	kb_ready = ready;
}

static int kb_get_report(const struct device *dev,
			 const uint8_t type, const uint8_t id, const uint16_t len,
			 uint8_t *const buf)
{
	LOG_WRN("Get Report not implemented, Type %u ID %u", type, id);

	return 0;
}

static int kb_set_report(const struct device *dev,
			 const uint8_t type, const uint8_t id, const uint16_t len,
			 const uint8_t *const buf)
{
	if (type != HID_REPORT_TYPE_OUTPUT) {
		LOG_WRN("Unsupported report type");
		return -ENOTSUP;
	}
/*
	for (unsigned int i = 0; i < ARRAY_SIZE(kb_leds); i++) {
		if (kb_leds[i].port == NULL) {
			continue;
		}

		(void)gpio_pin_set_dt(&kb_leds[i], buf[0] & BIT(i));
	}
*/
	return 0;
}

/* Idle duration is stored but not used to calculate idle reports. */
static void kb_set_idle(const struct device *dev,
			const uint8_t id, const uint32_t duration)
{
	LOG_INF("Set Idle %u to %u", id, duration);
	kb_duration = duration;
}

static uint32_t kb_get_idle(const struct device *dev, const uint8_t id)
{
	LOG_INF("Get Idle %u to %u", id, kb_duration);
	return kb_duration;
}

static void kb_set_protocol(const struct device *dev, const uint8_t proto)
{
	LOG_INF("Protocol changed to %s",
		proto == 0U ? "Boot Protocol" : "Report Protocol");
}

static void kb_output_report(const struct device *dev, const uint16_t len,
			     const uint8_t *const buf)
{
	LOG_HEXDUMP_DBG(buf, len, "o.r.");
	kb_set_report(dev, HID_REPORT_TYPE_OUTPUT, 0U, len, buf);
}

struct hid_device_ops kb_ops = {
	.iface_ready = kb_iface_ready,
	.get_report = kb_get_report,
	.set_report = kb_set_report,
	.set_idle = kb_set_idle,
	.get_idle = kb_get_idle,
	.set_protocol = kb_set_protocol,
	.output_report = kb_output_report,
};

/* doc device msg-cb start */
static void msg_cb(struct usbd_context *const usbd_ctx,
		   const struct usbd_msg *const msg)
{
	LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));

	if (msg->type == USBD_MSG_CONFIGURATION) {
		LOG_INF("\tConfiguration value %d", msg->status);
	}

	if (usbd_can_detect_vbus(usbd_ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(usbd_ctx)) {
				LOG_ERR("Failed to enable device support");
			}
		}

		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(usbd_ctx)) {
				LOG_ERR("Failed to disable device support");
			}
		}
	}
}
static K_SEM_DEFINE(ep_write_sem, 0, 1);
static void int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	k_sem_give(&ep_write_sem);
}
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usb_hid.h>
static const struct hid_ops mouse_ops = {
	.int_in_ready = int_in_ready_cb,
};
#define MOUSE_BTN_LEFT		0
#define MOUSE_BTN_RIGHT		1

enum mouse_report_idx {
	MOUSE_BTN_REPORT_IDX = 0,
	MOUSE_X_REPORT_IDX = 1,
	MOUSE_Y_REPORT_IDX = 2,
	MOUSE_WHEEL_REPORT_IDX = 3,
	MOUSE_REPORT_COUNT = 4,
};

struct k_msgq mouse_msgq;
struct k_msgq keyboard_msgq;

//K_MSGQ_DEFINE(mouse_msgq, MOUSE_REPORT_COUNT, 2, 1);

/* doc device msg-cb end */
static const uint8_t hid_report_desc_mouse[] = HID_MOUSE_REPORT_DESC(2);



#define HID_THREAD_STACK_SIZE 1024
#define HID_THREAD_PRIORITY 7
static char hid_mouse_msgq_buffer[10*MOUSE_REPORT_COUNT]; 
static char hid_keyboard_msgq_buffer[10*KB_REPORT_COUNT]; 
static void hid_keyboard_thread_fn(void)
{
	int ret;
	uint8_t tmp[KB_REPORT_COUNT];

	k_msgq_init(&keyboard_msgq, hid_keyboard_msgq_buffer, 8, 10);
	while(1) {
		UDC_STATIC_BUF_DEFINE(report, KB_REPORT_COUNT);

		k_msgq_get(&keyboard_msgq, &tmp, K_FOREVER);
		for (int i = 0; i < ARRAY_SIZE(tmp); ++i) {
			//printk(" 0x%x", tmp[i]);
			report[i] = tmp[i];
		}
		//printk("\n");
		hid_device_submit_report(hid_dev_keyboard, KB_REPORT_COUNT, report);
	}
}

static void hid_mouse_thread_fn(void)
{
	int ret;
	uint8_t tmp[MOUSE_REPORT_COUNT];

	k_msgq_init(&mouse_msgq, hid_mouse_msgq_buffer, 4, 10);
	while(1) {
		//printk("hid_mouse_thread_fn\n");
		UDC_STATIC_BUF_DEFINE(report, MOUSE_REPORT_COUNT);

		k_msgq_get(&mouse_msgq, &tmp, K_FOREVER);
		for (int i = 0; i < ARRAY_SIZE(tmp); ++i) {
			//printk(" 0x%x", tmp[i]);
		}
		//printk("\n");
		report[0] = tmp[0];
		report[1] = tmp[1];
		report[2] = tmp[2];
		report[3] = tmp[3];

		ret = hid_int_ep_write(hid_dev_mouse, report, MOUSE_REPORT_COUNT, NULL);
		if (ret) {
			LOG_ERR("HID write error, %d", ret);
		} else {
			k_sem_take(&ep_write_sem, K_FOREVER);
		}

	}
}
K_THREAD_DEFINE(hid_keyboard_thread, HID_THREAD_STACK_SIZE,
		(k_thread_entry_t)hid_keyboard_thread_fn, NULL, NULL, NULL,
		HID_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(hid_mouse_thread, HID_THREAD_STACK_SIZE,
		(k_thread_entry_t)hid_mouse_thread_fn, NULL, NULL, NULL,
		HID_THREAD_PRIORITY, 0, 0);

int audio_usb_init(void)
{
	int ret;
	LOG_INF("USB audio init");

	struct usbd_context *sample_usbd;

	usbd_uac2_set_ops(headset, &usb_audio_ops, &main_ctx);

	hid_dev_keyboard = DEVICE_DT_GET(DT_NODELABEL(hid_dev_0));
	if (!device_is_ready(hid_dev_keyboard)) {
		LOG_ERR("HID Device is not ready");
		return -EIO;
	}

	ret = hid_device_register(hid_dev_keyboard,
		hid_report_desc_keyboard, sizeof(hid_report_desc_keyboard),
				  &kb_ops);
	if (ret != 0) {
		LOG_ERR("Failed to register HID Device, %d", ret);
		return ret;
	}

	hid_dev_mouse = DEVICE_DT_GET(DT_NODELABEL(hid_dev_1));
	if (!device_is_ready(hid_dev_mouse)) {
		LOG_ERR("HID Device is not ready");
		return -EIO;
	}

	usb_hid_register_device(hid_dev_mouse,
		hid_report_desc_mouse, sizeof(hid_report_desc_mouse),
		&mouse_ops);

	usb_hid_init(hid_dev_mouse);


	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		return -ENODEV;
	}

	ret = usbd_enable(sample_usbd);
	if (ret) {
		return ret;
	}

	return 0;
}
