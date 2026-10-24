/*
 * Copyright (c) 2022-2026 Nordic Semiconductor ASA
 * Copyright (c) 2024 Demant A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Audio output: decodes the LC3 frames received on a CIS or on up to two BISes and plays them on
 * a TLV320 DAC over I2S. The TLV320 is the I2S master. Drift between the audio source and the DAC
 * clock is compensated by trimming the TLV320 PLL.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/autoconf.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

#include <nrfx_i2s.h>
#include <lc3.h>

#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */

#include "nrf54l15_audio_headset.h"

#define I2S_NL DT_NODELABEL(i2s20)
PINCTRL_DT_DEFINE(I2S_NL);
static nrfx_i2s_t i2s_inst = NRFX_I2S_INSTANCE(NRF_I2S20);
static nrfx_i2s_config_t cfg = {
	/* Pins are configured by pinctrl. */
	.skip_gpio_cfg = true,
	.skip_psel_cfg = true,
	.irq_priority = DT_IRQ(I2S_NL, priority),
	.mode = NRF_I2S_MODE_SLAVE,
	.format = NRF_I2S_FORMAT_I2S,
	.alignment = NRF_I2S_ALIGN_LEFT,
	.sample_width = NRF_I2S_SWIDTH_16BIT,
	.channels = NRF_I2S_CHANNELS_STEREO,
	.prescalers = {
		.ratio = NRF_I2S_RATIO_64X,
		.mck_setup = NRF_I2S_MCK_32MDIV2,
	},
};

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(tlv320));
/* The TLV320 reset line is connected to the LED3 pin */
static const struct gpio_dt_spec codec_rst = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);

/* TLV320 DAC digital volume range, in 0.5 dB steps */
#define TLV320_VOLUME_MAX 0    /* 0 dB */
#define TLV320_VOLUME_MIN -100 /* -50 dB */

#define I2S_SAMPLES_NUM 48 /* samples per 1ms block */
static uint16_t i2s_tx_buf_a[I2S_SAMPLES_NUM * 2] __aligned(4); /* 2 channels, 16 bits each */
static uint16_t i2s_tx_buf_b[I2S_SAMPLES_NUM * 2] __aligned(4); /* 2 channels, 16 bits each */
static uint16_t i2s_rx_buf_a[I2S_SAMPLES_NUM * 2] __aligned(4); /* 2 channels, 16 bits each */
static uint16_t i2s_rx_buf_b[I2S_SAMPLES_NUM * 2] __aligned(4); /* 2 channels, 16 bits each */
#define I2S_BLOCK_BYTES (I2S_SAMPLES_NUM * 2 * sizeof(uint16_t)) /* One 1 ms I2S block */
/* 80 ms of audio: the drift target (up to 40 ms) plus one ISO interval of frames (up to 30 ms)
 * plus 10 ms for a burst that arrives early. Covers ISO intervals of 10, 20 and 30 ms.
 */
#define BUFFER_SPACE 80 /* I2S blocks */
RING_BUF_DECLARE(i2s_tx_ring_buf, I2S_BLOCK_BYTES * BUFFER_SPACE);
/* I2S blocks not completely filled from the ring buffer, counted in the I2S ISR */
static atomic_t i2s_underrun_cnt;

/* Drift control keeps the lowest ring fill of each ISO interval at drift_target_ms, set per
 * stream by i2s_prime_setup() to max(DRIFT_TARGET_MIN_MS, ISO interval + one frame). The ring
 * then still holds audio if one channel arrives a whole ISO interval late.
 */
#define DRIFT_TARGET_MIN_MS 30
static uint32_t drift_target_ms = DRIFT_TARGET_MIN_MS;

/* A new stream starts playing only once the ring buffer holds i2s_prime_bytes: the drift target
 * plus one ISO interval of audio, set by i2s_prime_setup(). After the first interval has played,
 * the fill bottoms out at the target. Without priming, playback starts from an almost empty
 * buffer, which underruns until drift control slowly fills it.
 */
static uint32_t i2s_prime_bytes = (DRIFT_TARGET_MIN_MS + 10) * I2S_BLOCK_BYTES;
/* Cleared at stream start, set by the I2S ISR once i2s_prime_bytes are buffered */
static atomic_t i2s_primed;

#define MAX_SAMPLE_RATE       48000
#define MAX_FRAME_DURATION_US 10000
#define MAX_NUM_SAMPLES       ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)

#define PCM_OUT_SAMPLE_RATE 48000 /* Sample rate of the I2S output to the DAC */

#define OUT_CHAN_CNT   2 /* Left and right output */
#define OUT_CHAN_LEFT  0U
#define OUT_CHAN_RIGHT 1U

static lc3_decoder_t lc3_decoder[OUT_CHAN_CNT];
static lc3_decoder_mem_48k_t lc3_decoder_mem[OUT_CHAN_CNT];
/* PCM samples per channel decoded from one LC3 frame, set by lc3_decoders_setup() */
static int pcm_frame_samples = MAX_NUM_SAMPLES;
/* Expected SDU size, or 0 if unknown. Other sizes (e.g. multiple channels) are concealed. */
static uint16_t lc3_octets_per_frame;

/* LC3 configuration of the streams of a source */
struct lc3_params {
	int freq_hz;
	int frame_dur_us;
	uint16_t octets_per_frame; /* 0 if unknown */
};

/* The started streams of each source (CIS and BIS). All streams of a source use the codec
 * configuration of its first stream. Only the streams of the active source are played: the
 * source that started a stream last, or the other source when the active source has no streams
 * left. This way the audio follows the phone when it switches between CIS and BIS, even if the
 * streams of both are briefly running at the same time.
 */
static struct out_source {
	enum audio_out_src src;
	struct out_stream {
		const struct bt_bap_stream *stream;
		uint8_t chan; /* OUT_CHAN_LEFT or OUT_CHAN_RIGHT */
	} streams[OUT_CHAN_CNT];
	uint8_t cnt;
	struct lc3_params params;
} out_sources[] = {
	[AUDIO_OUT_SRC_CIS] = {.src = AUDIO_OUT_SRC_CIS},
	[AUDIO_OUT_SRC_BIS] = {.src = AUDIO_OUT_SRC_BIS},
};
/* The source being played, or NULL */
static struct out_source *out_active;
/* Stream start and stop may run in another thread than the reception of the audio data */
static K_MUTEX_DEFINE(out_mutex);

/* Output statistics, printed every REPORT_INTERVAL received SDUs (all streams together). The
 * counts cover the time since the previous report.
 */
#define REPORT_INTERVAL 500U

static struct {
	uint32_t qfull[OUT_CHAN_CNT]; /* Oldest packet dropped because the L/R queue was full */
	uint32_t drop[OUT_CHAN_CNT];  /* Packet dropped from the L/R queue to realign L and R */
	uint32_t decoded;   /* Frames decoded and written to the I2S ring buffer */
	uint32_t mono;      /* Frames decoded with only one of L/R available */
	uint32_t wait[OUT_CHAN_CNT];  /* Decodes held back because the L/R queue was empty */
	uint32_t plc[OUT_CHAN_CNT];   /* L/R frames concealed by LC3: lost or undecodable SDUs */
	uint32_t ring_full; /* Decoded frames dropped because the I2S ring buffer was full */
	uint32_t sdu_cnt;   /* SDUs since the last report */
} out_stats;

static void audio_i2s_set_next_buf(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	const nrfx_i2s_buffers_t i2s_buf = {.p_rx_buffer = rx_buf,
					    .p_tx_buffer = (uint32_t *)tx_buf,
					    .buffer_size = I2S_SAMPLES_NUM};

	int ret;

	ret = nrfx_i2s_next_buffers_set(&i2s_inst, &i2s_buf);
	if (ret != 0) {
		printk("Failed to set next buffers: %d\n", ret);
	}
}

static void i2s_comp_handler(nrfx_i2s_buffers_t const *released_bufs, uint32_t status)
{
	uint16_t *tx_buf;
	uint16_t *rx_buf;
	uint32_t read;

	if (status != NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED || released_bufs == NULL) {
		return;
	}

	if ((uint16_t *)released_bufs->p_tx_buffer == i2s_tx_buf_a) {
		tx_buf = i2s_tx_buf_a;
		rx_buf = i2s_rx_buf_a;
	} else if ((uint16_t *)released_bufs->p_tx_buffer == i2s_tx_buf_b) {
		tx_buf = i2s_tx_buf_b;
		rx_buf = i2s_rx_buf_b;
	} else {
		return;
	}

	/* Play silence, without consuming, until a new stream has buffered i2s_prime_bytes */
	if (!atomic_get(&i2s_primed)) {
		if (ring_buf_size_get(&i2s_tx_ring_buf) < i2s_prime_bytes) {
			memset(tx_buf, 0, I2S_BLOCK_BYTES);
			audio_i2s_set_next_buf((const uint8_t *)tx_buf, (uint32_t *)rx_buf);
			return;
		}
		atomic_set(&i2s_primed, 1);
	}

	/* On underrun, keep what was read and fill only the rest with silence */
	read = ring_buf_get(&i2s_tx_ring_buf, (uint8_t *)tx_buf, I2S_BLOCK_BYTES);
	if (read < I2S_BLOCK_BYTES) {
		atomic_inc(&i2s_underrun_cnt);
		memset((uint8_t *)tx_buf + read, 0, I2S_BLOCK_BYTES - read);
	}

	audio_i2s_set_next_buf((const uint8_t *)tx_buf, (uint32_t *)rx_buf);
}

static void audio_i2s_start(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	const nrfx_i2s_buffers_t i2s_buf = {.p_rx_buffer = rx_buf,
					    .p_tx_buffer = (uint32_t *)tx_buf,
					    .buffer_size = I2S_SAMPLES_NUM};

	int ret;

	/* Buffer size in 32-bit words */
	ret = nrfx_i2s_start(&i2s_inst, &i2s_buf, 0);
	if (ret != 0) {
		printk("Failed to start I2S: %d\n", ret);
	}
}

static int audio_i2s_init(void)
{
	int ret;

	ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(I2S_NL), PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		printk("Failed to apply pinctrl state: %d\n", ret);
		return ret;
	}

	IRQ_CONNECT(DT_IRQN(I2S_NL), DT_IRQ(I2S_NL, priority), nrfx_i2s_irq_handler, &i2s_inst,
		    0);
	irq_enable(DT_IRQN(I2S_NL));

	ret = nrfx_i2s_init(&i2s_inst, &cfg, i2s_comp_handler);
	if (ret != 0) {
		printk("Failed to initialize I2S: %d\n", ret);
		return ret;
	}

	return 0;
}

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

static void dac_i2c_write(const struct i2c_dt_spec *spec, uint8_t reg, uint8_t value)
{
	int ret;
	uint8_t config[2] = {reg, value};

	ret = i2c_write_dt(spec, config, sizeof(config));
	if (ret != 0) {
		printk("Failed to write to I2C device address %x at reg. %x\n", spec->addr, reg);
	}
}

static void tlv320_set_volume(int8_t vol)
{
	/* Page 0: left (register 65) and right (register 66) DAC digital volume */
	dac_i2c_write(&dev_i2c, 0x41, vol);
	dac_i2c_write(&dev_i2c, 0x42, vol);
}

static int tlv320_setup(void)
{
	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n", dev_i2c.bus->name);
		return -ENODEV;
	}

	printk("I2C bus %s is ready!\n", dev_i2c.bus->name);

	dac_i2c_write(&dev_i2c, 0x00, 0x00);

	dac_i2c_write(&dev_i2c, 0x01, 0x01);
	k_sleep(K_MSEC(10));

	dac_i2c_write(&dev_i2c, 0x04, 0x03 | (0b11 << 0));
	dac_i2c_write(&dev_i2c, 0x05, (0b001 << 4) | (0b0001 << 0));
	dac_i2c_write(&dev_i2c, 0x06, 0x05);
	dac_i2c_write(&dev_i2c, 0x07, 0x0E);
	dac_i2c_write(&dev_i2c, 0x08, 0xB0);

	dac_i2c_write(&dev_i2c, 0x05, (1 << 7) | (0b001 << 4) | (0b0001 << 0));
	k_sleep(K_MSEC(15));

	dac_i2c_write(&dev_i2c, 0x0B, 0x87);
	dac_i2c_write(&dev_i2c, 0x0C, 0x82);
	dac_i2c_write(&dev_i2c, 0x0D, 0x00);
	dac_i2c_write(&dev_i2c, 0x0E, 0x80);

	dac_i2c_write(&dev_i2c, 0x1B, 0x0C);
	dac_i2c_write(&dev_i2c, 0x1E, 0x84);
	dac_i2c_write(&dev_i2c, 0x1D, (0b01 << 0));
	dac_i2c_write(&dev_i2c, 0x3C, 0x01);
	dac_i2c_write(&dev_i2c, 0x74, 0x00);

	dac_i2c_write(&dev_i2c, 0x00, 0x01);
	dac_i2c_write(&dev_i2c, 0x1F, (0b00 << 3));
	dac_i2c_write(&dev_i2c, 0x21, (0b0111 << 3) | (0b11 << 1));
	dac_i2c_write(&dev_i2c, 0x23, 0x44);
	dac_i2c_write(&dev_i2c, 0x24, 0x80);
	dac_i2c_write(&dev_i2c, 0x25, 0x80);
	dac_i2c_write(&dev_i2c, 0x28, 0x06);
	dac_i2c_write(&dev_i2c, 0x29, 0x06);
	dac_i2c_write(&dev_i2c, 0x1F, 0xC0 | (0b00 << 3));

	k_sleep(K_MSEC(300));

	dac_i2c_write(&dev_i2c, 0x00, 0x00);
	dac_i2c_write(&dev_i2c, 0x3F, 0xD4);
	/* The volume is set by the VCP Volume Renderer, keep the DAC muted until then */
	dac_i2c_write(&dev_i2c, 0x40, 0x0C);
	dac_i2c_write(&dev_i2c, 0x00, 0x00);

	return 0;
}

struct recv_pkt_info {
	uint32_t sdu_ref_us;
	bool has_ts;    /* sdu_ref_us is valid (BT_ISO_FLAGS_TS) */
	bool bad_frame; /* Lost or invalid SDU, decoded with packet loss concealment */
	uint16_t size;
	uint8_t buf[CONFIG_BT_ISO_RX_MTU];
};

/* Jitter queue per channel, sized for each stream by jitter_buffer_setup():
 *   decode threshold = SDUs per ISO interval (N)
 *   size             = N * (1 + JITTER_BUFFER_EXTRA)
 * A stream delivers its SDUs per ISO interval, so a late channel falls behind in steps of N
 * SDUs. The extra space therefore counts ISO intervals: a channel may lag by up to
 * JITTER_BUFFER_EXTRA ISO intervals before the other channel is played alone
 * (see recv_locked()).
 */
#define JITTER_BUFFER_EXTRA   2  /* ISO intervals a late channel may lag */
#define JITTER_BUFFER_MAX     12 /* Slots per channel queue */
#define JITTER_BUFFER_DEFAULT 2  /* SDUs per ISO interval assumed if unknown */

/* The queues are re-initialized between streams, which is only safe without object core */
BUILD_ASSERT(!IS_ENABLED(CONFIG_OBJ_CORE_MSGQ), "k_msgq_init() is called more than once");

static char recv_pkt_msgq_l_buf[JITTER_BUFFER_MAX * sizeof(struct recv_pkt_info)] __aligned(4);
static char recv_pkt_msgq_r_buf[JITTER_BUFFER_MAX * sizeof(struct recv_pkt_info)] __aligned(4);
static struct k_msgq recv_pkt_msgq_l;
static struct k_msgq recv_pkt_msgq_r;
static uint8_t jitter_buffer_size;
static uint8_t jitter_buffer_check;
static uint8_t jitter_buffer_sdus = JITTER_BUFFER_DEFAULT; /* SDUs per ISO interval */

/* Size both queues for sdus_per_interval SDUs per ISO interval. Must only be called while no
 * stream is played.
 */
static void jitter_buffer_setup(uint32_t sdus_per_interval)
{
	uint32_t size = sdus_per_interval * (1U + JITTER_BUFFER_EXTRA);

	if (size > JITTER_BUFFER_MAX) {
		printk("Jitter buffer needs %u slots, limited to %u\n", size, JITTER_BUFFER_MAX);
		size = JITTER_BUFFER_MAX;
	}

	jitter_buffer_size = size;
	jitter_buffer_check = MIN(sdus_per_interval, size);
	jitter_buffer_sdus = MIN(sdus_per_interval, UINT8_MAX);
	k_msgq_init(&recv_pkt_msgq_l, recv_pkt_msgq_l_buf, sizeof(struct recv_pkt_info), size);
	k_msgq_init(&recv_pkt_msgq_r, recv_pkt_msgq_r_buf, sizeof(struct recv_pkt_info), size);
}

/* Max SDU timestamp difference between L and R packets that are played together */
#define LR_SYNC_MAX_DIFF_US 2000

/* TLV320 PLL D values that set the DAC sample rate */
#define TLV320_PLL_D_FAST    3802 /* 48037.5 Hz */
#define TLV320_PLL_D_NOMINAL 3760 /* 48000 Hz */
#define TLV320_PLL_D_SLOW    3718 /* 47962.5 Hz */

/* The DAC rate changes when an ISO interval's lowest fill leaves drift_target_ms +/-
 * DRIFT_BAND_MS, and returns to nominal once the low point is back at the target.
 */
#define DRIFT_BAND_MS 3

static void tlv320_set_pll_d(uint16_t d)
{
	/* Page 0: PLL D-value MSB (register 7) and LSB (register 8) */
	dac_i2c_write(&dev_i2c, 0x07, (d >> 8) & 0x3F);
	dac_i2c_write(&dev_i2c, 0x08, d & 0xFF);
}

/* Adjust the DAC sample rate to keep the I2S ring buffer's low point at drift_target_ms. The
 * PLL is only written when the rate changes.
 */
static enum drift_state {
	DRIFT_NOMINAL,
	DRIFT_FAST,
	DRIFT_SLOW,
} drift_state = DRIFT_NOMINAL;

/* Audio in the I2S ring buffer, in ms (one I2S block is 1 ms) */
static uint32_t i2s_ring_fill_ms(void)
{
	return ring_buf_size_get(&i2s_tx_ring_buf) / I2S_BLOCK_BYTES;
}

/* Lowest fill (ms) in the current ISO interval and frames decoded in it, see below */
static uint32_t drift_fill_min = UINT32_MAX;
static uint8_t drift_frame_cnt;
/* Fill (ms) drift control last decided on, for the statistics report */
static uint32_t drift_fill_last;

static void drift_state_set(enum drift_state state)
{
	drift_state = state;
	if (state == DRIFT_FAST) {
		tlv320_set_pll_d(TLV320_PLL_D_FAST);
	} else if (state == DRIFT_SLOW) {
		tlv320_set_pll_d(TLV320_PLL_D_SLOW);
	} else {
		tlv320_set_pll_d(TLV320_PLL_D_NOMINAL);
	}
}

static void drift_control_update(void)
{
	enum drift_state state = drift_state;
	enum drift_state new_state = state;
	uint32_t fill_ms;

	/* The fill only rises while the I2S ISR is priming; nothing to correct yet */
	if (!atomic_get(&i2s_primed)) {
		drift_fill_min = UINT32_MAX;
		drift_frame_cnt = 0U;
		return;
	}

	/* Each decoded frame adds one frame of audio and the DAC drains it between ISO
	 * intervals, so with N SDUs per interval the fill is a sawtooth N frames high. Decide once
	 * per interval, on its lowest point (just before the interval's first frame), so the rate
	 * does not flip inside every interval.
	 */
	drift_fill_min = MIN(drift_fill_min, i2s_ring_fill_ms());
	if (++drift_frame_cnt < jitter_buffer_sdus) {
		return;
	}

	fill_ms = drift_fill_min;
	drift_fill_last = fill_ms;
	drift_fill_min = UINT32_MAX;
	drift_frame_cnt = 0U;

	switch (state) {
	case DRIFT_NOMINAL:
		if (fill_ms > drift_target_ms + DRIFT_BAND_MS) {
			new_state = DRIFT_FAST;
		} else if (fill_ms + DRIFT_BAND_MS < drift_target_ms) {
			new_state = DRIFT_SLOW;
		}
		break;
	case DRIFT_FAST:
		if (fill_ms <= drift_target_ms) {
			new_state = DRIFT_NOMINAL;
		}
		break;
	case DRIFT_SLOW:
		if (fill_ms >= drift_target_ms) {
			new_state = DRIFT_NOMINAL;
		}
		break;
	}

	if (new_state != state) {
		drift_state_set(new_state);
	}
}

static struct k_msgq *recv_pkt_msgq_get(uint8_t chan)
{
	return chan == OUT_CHAN_LEFT ? &recv_pkt_msgq_l : &recv_pkt_msgq_r;
}

static void recv_pkt_enqueue(uint8_t chan, const struct recv_pkt_info *pkt_info)
{
	struct k_msgq *msgq = recv_pkt_msgq_get(chan);

	if (k_msgq_put(msgq, pkt_info, K_NO_WAIT) == -ENOMSG) {
		struct recv_pkt_info oldest;

		/* Queue full: drop the oldest packet to make room for the newest */
		out_stats.qfull[chan]++;
		(void)k_msgq_get(msgq, &oldest, K_NO_WAIT);
		(void)k_msgq_put(msgq, pkt_info, K_NO_WAIT);
	}
}

static void out_stats_report(void)
{
	static const char *const drift_name[] = {
		[DRIFT_NOMINAL] = "nominal",
		[DRIFT_FAST] = "fast",
		[DRIFT_SLOW] = "slow",
	};

	printk("OUT: decoded %u mono %u | plc L %u R %u | wait L %u R %u | realign drop L %u R %u | "
	       "qfull L %u R %u\n",
	       out_stats.decoded, out_stats.mono, out_stats.plc[0], out_stats.plc[1],
	       out_stats.wait[0], out_stats.wait[1], out_stats.drop[0], out_stats.drop[1],
	       out_stats.qfull[0], out_stats.qfull[1]);
	printk("I2S: ringfull %u underrun %u | fill %u ms now, %u ms interval low, target %u ms | "
	       "rate %s\n",
	       out_stats.ring_full, (uint32_t)atomic_clear(&i2s_underrun_cnt), i2s_ring_fill_ms(),
	       drift_fill_last, drift_target_ms, drift_name[drift_state]);

	memset(&out_stats, 0, sizeof(out_stats));
}

/* Decode one frame and queue it for I2S. l or r is NULL when that channel is missing; the
 * channel that is present is then copied to both outputs.
 */
static void frame_play(const struct recv_pkt_info *l, const struct recv_pkt_info *r)
{
	/* Zeroed so a failed decode outputs silence instead of stack data */
	int16_t audio_buf[2 * MAX_NUM_SAMPLES] = {0};
	uint32_t frame_bytes;
	int err;

	drift_control_update();

	if (l != NULL) {
		err = lc3_decode(lc3_decoder[OUT_CHAN_LEFT], l->bad_frame ? NULL : l->buf, l->size,
				 LC3_PCM_FORMAT_S16, audio_buf + 0, 2);
		if (err < 0) {
			printk("L: LC3 decode failed: %d\n", err);
		} else if (err > 0) {
			out_stats.plc[0]++;
		}
	}
	if (r != NULL) {
		err = lc3_decode(lc3_decoder[OUT_CHAN_RIGHT], r->bad_frame ? NULL : r->buf, r->size,
				 LC3_PCM_FORMAT_S16, audio_buf + 1, 2);
		if (err < 0) {
			printk("R: LC3 decode failed: %d\n", err);
		} else if (err > 0) {
			out_stats.plc[1]++;
		}
	}

	/* Mono: copy the received channel to the other output */
	if (l != NULL && r == NULL) {
		out_stats.mono++;
		for (int j = 0; j < pcm_frame_samples; j++) {
			audio_buf[2 * j + 1] = audio_buf[2 * j];
		}
	} else if (r != NULL && l == NULL) {
		out_stats.mono++;
		for (int j = 0; j < pcm_frame_samples; j++) {
			audio_buf[2 * j] = audio_buf[2 * j + 1];
		}
	}

	/* Drop the whole frame rather than writing part of it when the buffer is full */
	frame_bytes = pcm_frame_samples * 2 * sizeof(int16_t);
	if (ring_buf_space_get(&i2s_tx_ring_buf) < frame_bytes) {
		out_stats.ring_full++;
		return;
	}

	(void)ring_buf_put(&i2s_tx_ring_buf, (uint8_t *)audio_buf, frame_bytes);
	out_stats.decoded++;
}

static int out_stream_find(const struct out_source *source, const struct bt_bap_stream *stream)
{
	for (uint8_t i = 0U; i < source->cnt; i++) {
		if (source->streams[i].stream == stream) {
			return i;
		}
	}

	return -1;
}

/* Called with out_mutex held, for a stream of the active source */
static void recv_locked(uint8_t chan, const struct bt_iso_recv_info *info,
			const struct net_buf *buf)
{
	struct recv_pkt_info pkt_info = {0};
	struct recv_pkt_info pkt_info_l = {0};
	struct recv_pkt_info pkt_info_r = {0};
	struct k_msgq *own_q = recv_pkt_msgq_get(chan);
	struct k_msgq *other_q = recv_pkt_msgq_get(chan == OUT_CHAN_LEFT ? OUT_CHAN_RIGHT
									  : OUT_CHAN_LEFT);
	const bool stereo = out_active->cnt > 1U;
	bool has_l;
	bool has_r;

	pkt_info.sdu_ref_us = info->ts;
	pkt_info.has_ts = (info->flags & BT_ISO_FLAGS_TS) != 0;
	pkt_info.bad_frame = buf->len == 0U || (info->flags & BT_ISO_FLAGS_VALID) == 0 ||
			     buf->len > sizeof(pkt_info.buf) ||
			     (lc3_octets_per_frame != 0U && buf->len != lc3_octets_per_frame);
	if (!pkt_info.bad_frame) {
		pkt_info.size = buf->len;
		memcpy(pkt_info.buf, buf->data, buf->len);
	}

	/* Stereo, this queue is full and the other channel has nothing: the other channel is
	 * more than JITTER_BUFFER_EXTRA ISO intervals late. Play this channel's oldest packet
	 * alone to make room, rather than discarding it.
	 */
	if (stereo && k_msgq_num_free_get(own_q) == 0U && k_msgq_num_used_get(other_q) == 0U) {
		struct recv_pkt_info *oldest = chan == OUT_CHAN_LEFT ? &pkt_info_l : &pkt_info_r;

		(void)k_msgq_get(own_q, oldest, K_NO_WAIT);
		frame_play(chan == OUT_CHAN_LEFT ? oldest : NULL,
			   chan == OUT_CHAN_LEFT ? NULL : oldest);
	}

	recv_pkt_enqueue(chan, &pkt_info);

	if (k_msgq_num_used_get(&recv_pkt_msgq_l) < jitter_buffer_check &&
	    k_msgq_num_used_get(&recv_pkt_msgq_r) < jitter_buffer_check) {
		return;
	}

	/* Only one queue has data for a mono (single stream) source */
	has_l = k_msgq_peek(&recv_pkt_msgq_l, &pkt_info_l) == 0;
	has_r = k_msgq_peek(&recv_pkt_msgq_r, &pkt_info_r) == 0;

	/* In stereo, an empty queue means that channel's SDUs are late: wait for them instead of
	 * playing the other channel alone, which would make the late packet look old and get it
	 * dropped. If the wait outgrows the queue, the overflow case above plays the oldest
	 * packet alone.
	 */
	if (has_l != has_r && stereo) {
		out_stats.wait[has_l ? OUT_CHAN_RIGHT : OUT_CHAN_LEFT]++;
		return;
	}

	/* Align L and R on the SDU timestamps, unless an SDU has no timestamp (ts is then 0).
	 * The signed difference handles the 32-bit us timestamp wrap (~71.6 min).
	 */
	if (has_l && has_r && pkt_info_l.has_ts && pkt_info_r.has_ts) {
		int32_t ts_diff = (int32_t)(pkt_info_l.sdu_ref_us - pkt_info_r.sdu_ref_us);

		if (ts_diff > LR_SYNC_MAX_DIFF_US) {
			/* R packet is older than L: drop it */
			(void)k_msgq_get(&recv_pkt_msgq_r, &pkt_info_r, K_NO_WAIT);
			out_stats.drop[OUT_CHAN_RIGHT]++;
			return;
		}

		if (ts_diff < -LR_SYNC_MAX_DIFF_US) {
			/* L packet is older than R: drop it */
			(void)k_msgq_get(&recv_pkt_msgq_l, &pkt_info_l, K_NO_WAIT);
			out_stats.drop[OUT_CHAN_LEFT]++;
			return;
		}
	}

	if (has_l) {
		(void)k_msgq_get(&recv_pkt_msgq_l, &pkt_info_l, K_NO_WAIT);
	}
	if (has_r) {
		(void)k_msgq_get(&recv_pkt_msgq_r, &pkt_info_r, K_NO_WAIT);
	}

	frame_play(has_l ? &pkt_info_l : NULL, has_r ? &pkt_info_r : NULL);
}

void audio_out_recv(const struct bt_bap_stream *stream, const struct bt_iso_recv_info *info,
		    const struct net_buf *buf)
{
	int idx;

	k_mutex_lock(&out_mutex, K_FOREVER);

	idx = out_active != NULL ? out_stream_find(out_active, stream) : -1;
	if (idx >= 0) {
		recv_locked(out_active->streams[idx].chan, info, buf);

		if (++out_stats.sdu_cnt >= REPORT_INTERVAL) {
			out_stats_report();
		}
	}

	k_mutex_unlock(&out_mutex);
}

/* Drift target and priming level for the next stream. The target is max(30 ms, ISO interval +
 * one frame), so the ring covers a channel arriving one whole ISO interval late. Playback
 * starts at the target plus one ISO interval, kept one frame below the ring size so the frame
 * that completes it still fits.
 */
static void i2s_prime_setup(void)
{
	uint32_t frame_bytes = pcm_frame_samples * 2 * sizeof(int16_t);
	uint32_t frame_ms = frame_bytes / I2S_BLOCK_BYTES;
	uint32_t interval_ms = jitter_buffer_sdus * frame_ms;
	uint32_t max = ring_buf_capacity_get(&i2s_tx_ring_buf) - frame_bytes;

	drift_target_ms = MAX(DRIFT_TARGET_MIN_MS, interval_ms + frame_ms);
	i2s_prime_bytes = MIN((drift_target_ms + interval_ms) * I2S_BLOCK_BYTES, max);
	printk("I2S: drift target %u ms, playback starts after %u ms buffered\n", drift_target_ms,
	       i2s_prime_bytes / I2S_BLOCK_BYTES);
}

static int lc3_decoders_setup(int freq_hz, int frame_dur_us)
{
	for (size_t i = 0U; i < ARRAY_SIZE(lc3_decoder); i++) {
		lc3_decoder[i] = lc3_setup_decoder(frame_dur_us, freq_hz, PCM_OUT_SAMPLE_RATE,
						   &lc3_decoder_mem[i]);
		if (lc3_decoder[i] == NULL) {
			printk("Unsupported LC3 configuration: %d Hz, %d us frames\n", freq_hz,
			       frame_dur_us);
			return -EINVAL;
		}
	}

	pcm_frame_samples = lc3_frame_samples(frame_dur_us, PCM_OUT_SAMPLE_RATE);

	return 0;
}

static uint32_t sdus_per_iso_interval_get(const struct bt_bap_stream *stream)
{
	struct bt_iso_info iso_info;

	if (stream->iso == NULL || stream->qos == NULL || stream->qos->interval == 0U ||
	    bt_iso_chan_get_info(stream->iso, &iso_info) != 0) {
		return JITTER_BUFFER_DEFAULT;
	}

	return DIV_ROUND_UP(BT_CONN_INTERVAL_TO_US(iso_info.iso_interval), stream->qos->interval);
}

static const char *src_str(enum audio_out_src src)
{
	return src == AUDIO_OUT_SRC_CIS ? "CIS" : "BIS";
}

static int lc3_params_get(const struct bt_bap_stream *stream, struct lc3_params *params)
{
	const struct bt_audio_codec_cfg *codec_cfg = stream->codec_cfg;
	int ret;

	if (codec_cfg == NULL || codec_cfg->id != BT_HCI_CODING_FORMAT_LC3) {
		printk("Only the LC3 codec is supported\n");
		return -EINVAL;
	}

	ret = bt_audio_codec_cfg_get_freq(codec_cfg);
	if (ret >= 0) {
		ret = bt_audio_codec_cfg_freq_to_freq_hz(ret);
	}
	if (ret < 0) {
		printk("Could not get sampling frequency: %d\n", ret);
		return ret;
	}
	params->freq_hz = ret;

	ret = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
	if (ret >= 0) {
		ret = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);
	}
	if (ret < 0) {
		printk("Could not get frame duration: %d\n", ret);
		return ret;
	}
	params->frame_dur_us = ret;

	ret = bt_audio_codec_cfg_get_octets_per_frame(codec_cfg);
	params->octets_per_frame = ret > 0 ? ret : 0U;

	return 0;
}

static bool lc3_params_eq(const struct lc3_params *a, const struct lc3_params *b)
{
	return a->freq_hz == b->freq_hz && a->frame_dur_us == b->frame_dur_us &&
	       a->octets_per_frame == b->octets_per_frame;
}

/* Set up the decoders, jitter buffer and I2S priming to play the streams of a source. Restarts
 * the playback from an empty jitter buffer.
 */
static int out_setup(const struct out_source *source)
{
	const struct lc3_params *params = &source->params;
	uint32_t sdus_per_interval;
	int ret;

	ret = lc3_decoders_setup(params->freq_hz, params->frame_dur_us);
	if (ret != 0) {
		return ret;
	}

	lc3_octets_per_frame = params->octets_per_frame;

	sdus_per_interval = sdus_per_iso_interval_get(source->streams[0].stream);
	jitter_buffer_setup(sdus_per_interval);
	i2s_prime_setup();

	printk("Audio out (%s): %d Hz, %d us frames, %u octets, %u SDUs per ISO interval, jitter "
	       "buffer %u slots, decode at %u\n",
	       src_str(source->src), params->freq_hz, params->frame_dur_us,
	       params->octets_per_frame, sdus_per_interval, jitter_buffer_size,
	       jitter_buffer_check);

	/* Count from the stream start, not the I2S underruns while idle */
	memset(&out_stats, 0, sizeof(out_stats));
	(void)atomic_clear(&i2s_underrun_cnt);
	drift_fill_min = UINT32_MAX;
	drift_frame_cnt = 0U;
	if (drift_state != DRIFT_NOMINAL) {
		drift_state_set(DRIFT_NOMINAL);
	}

	/* Buffer i2s_prime_bytes before playing, see i2s_comp_handler() */
	(void)atomic_clear(&i2s_primed);

	return 0;
}

/* Stop playing: the remaining audio in the I2S ring buffer is played out, then silence */
static void out_deactivate(void)
{
	out_active = NULL;

	ARRAY_FOR_EACH(lc3_decoder, i) {
		lc3_decoder[i] = NULL;
	}

	k_msgq_purge(&recv_pkt_msgq_l);
	k_msgq_purge(&recv_pkt_msgq_r);
}

/* Play the streams of source instead of the streams of the active source, if any */
static void out_activate(struct out_source *source)
{
	int err;

	if (out_active != NULL) {
		printk("Audio out: switching from %s to %s\n", src_str(out_active->src),
		       src_str(source->src));
	}

	err = out_setup(source);
	if (err != 0) {
		printk("Audio out: cannot play %s: %d\n", src_str(source->src), err);
		out_deactivate();

		return;
	}

	out_active = source;
}

/* Use the stream's channel allocation if it is exactly front left or right and that output is
 * free; otherwise the first free output
 */
static uint8_t out_chan_get(const struct out_source *source, const struct bt_bap_stream *stream)
{
	const enum bt_audio_location front_lr =
		BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT;
	bool used[OUT_CHAN_CNT] = {false};
	enum bt_audio_location loc;

	for (uint8_t i = 0U; i < source->cnt; i++) {
		used[source->streams[i].chan] = true;
	}

	if (stream->codec_cfg != NULL &&
	    bt_audio_codec_cfg_get_chan_allocation(stream->codec_cfg, &loc, true) == 0) {
		if ((loc & front_lr) == BT_AUDIO_LOCATION_FRONT_RIGHT && !used[OUT_CHAN_RIGHT]) {
			return OUT_CHAN_RIGHT;
		}

		if ((loc & front_lr) == BT_AUDIO_LOCATION_FRONT_LEFT && !used[OUT_CHAN_LEFT]) {
			return OUT_CHAN_LEFT;
		}
	}

	return used[OUT_CHAN_LEFT] ? OUT_CHAN_RIGHT : OUT_CHAN_LEFT;
}

/* Check that a new stream of source can be played */
static int out_stream_check(const struct out_source *source, const struct bt_bap_stream *stream,
			    struct lc3_params *params)
{
	int err;

	if (out_stream_find(source, stream) >= 0) {
		return -EALREADY;
	}

	if (source->cnt == ARRAY_SIZE(source->streams)) {
		printk("Audio out can only play %zu %s streams, not playing stream %p\n",
		       ARRAY_SIZE(source->streams), src_str(source->src), stream);
		return -ENOMEM;
	}

	err = lc3_params_get(stream, params);
	if (err != 0) {
		return err;
	}

	if (source->cnt == 0U) {
		/* Check now that the decoders support it, so that switching cannot fail later */
		if (lc3_frame_samples(params->frame_dur_us, params->freq_hz) < 0) {
			printk("Unsupported LC3 configuration: %d Hz, %d us frames\n",
			       params->freq_hz, params->frame_dur_us);
			return -EINVAL;
		}
	} else if (!lc3_params_eq(params, &source->params)) {
		/* E.g. the right CIS, which is decoded with the configuration of the left CIS */
		printk("Stream %p has another LC3 configuration (%d Hz, %d us, %u octets) than the "
		       "other %s stream, not playing it\n",
		       stream, params->freq_hz, params->frame_dur_us, params->octets_per_frame,
		       src_str(source->src));
		return -EINVAL;
	}

	return 0;
}

int audio_out_stream_start(enum audio_out_src src, const struct bt_bap_stream *stream)
{
	struct out_source *source = &out_sources[src];
	struct lc3_params params;
	int err;

	k_mutex_lock(&out_mutex, K_FOREVER);

	err = out_stream_check(source, stream, &params);
	if (err == 0) {
		struct out_stream *out_stream = &source->streams[source->cnt];

		if (source->cnt == 0U) {
			source->params = params;
		}

		out_stream->stream = stream;
		out_stream->chan = out_chan_get(source, stream);
		source->cnt++;

		printk("Audio out: %s stream %p on the %s output\n", src_str(src), stream,
		       out_stream->chan == OUT_CHAN_LEFT ? "left" : "right");

		/* The source that started last is played; another stream of the active source
		 * just joins it
		 */
		if (out_active != source) {
			out_activate(source);
		}
	}

	k_mutex_unlock(&out_mutex);

	return err;
}

void audio_out_stream_stop(const struct bt_bap_stream *stream)
{
	k_mutex_lock(&out_mutex, K_FOREVER);

	ARRAY_FOR_EACH_PTR(out_sources, source) {
		const int idx = out_stream_find(source, stream);
		uint8_t chan;

		if (idx < 0) {
			continue;
		}

		chan = source->streams[idx].chan;
		source->cnt--;
		source->streams[idx] = source->streams[source->cnt];

		printk("Audio out: %s stream %p stopped\n", src_str(source->src), stream);

		if (source == out_active) {
			k_msgq_purge(recv_pkt_msgq_get(chan));
		}

		if (source == out_active && source->cnt == 0U) {
			struct out_source *other = &out_sources[source->src == AUDIO_OUT_SRC_CIS
									  ? AUDIO_OUT_SRC_BIS
									  : AUDIO_OUT_SRC_CIS];

			if (other->cnt > 0U) {
				/* Fall back to the streams that are still running */
				out_activate(other);
			} else {
				printk("Audio out: idle\n");
				out_deactivate();
			}
		}

		break;
	}

	k_mutex_unlock(&out_mutex);
}

void audio_out_volume_set(uint8_t volume, bool mute)
{
	const int8_t vol = TLV320_VOLUME_MIN +
			   ((TLV320_VOLUME_MAX - TLV320_VOLUME_MIN) * volume) / UINT8_MAX;

	tlv320_set_volume(vol);
	/* Page 0: DAC left and right mute (register 64) */
	dac_i2c_write(&dev_i2c, 0x40, (mute || volume == 0U) ? 0x0C : 0x00);
}

int audio_out_init(void)
{
	int err;

	err = gpio_pin_configure_dt(&codec_rst, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		printk("Failed to configure the TLV320 reset pin: %d\n", err);
		return err;
	}

	err = clocks_start();
	if (err != 0) {
		return err;
	}

	/* Reset the TLV320 */
	k_sleep(K_MSEC(1000));
	gpio_pin_set_dt(&codec_rst, 1);
	k_sleep(K_MSEC(10));

	/* Valid queues before any Bluetooth callback can purge them */
	jitter_buffer_setup(JITTER_BUFFER_DEFAULT);

	err = tlv320_setup();
	if (err != 0) {
		return err;
	}

	err = audio_i2s_init();
	if (err != 0) {
		return err;
	}

	audio_i2s_start((uint8_t *)i2s_tx_buf_a, (uint32_t *)i2s_rx_buf_a);
	audio_i2s_set_next_buf((const uint8_t *)i2s_tx_buf_b, (uint32_t *)i2s_rx_buf_b);

	return 0;
}
