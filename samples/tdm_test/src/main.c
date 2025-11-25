#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/printk.h>
#include <math.h>

/* ---------------- Basic Parameters ---------------- */
#define SAMPLE_RATE_HZ      48000
#define TONE_HZ             400

/* I2S: 16-bit stereo, interleaved (L, R, L, R, ...) */
#define CHANNELS            2
#define WORD_BITS           16
#define BYTES_PER_SAMPLE    (WORD_BITS / 8)               /* per channel */
#define FRAME_BYTES         (CHANNELS * BYTES_PER_SAMPLE) /* L+R */

/* 400 Hz → 48000 / 400 = 120 samples per cycle */
#define SINE_TABLE_LEN      (SAMPLE_RATE_HZ / TONE_HZ)

/* Each block contains one full sine-wave period */
#define FRAMES_PER_BLOCK    (SINE_TABLE_LEN)
#define BLOCK_SIZE          (FRAMES_PER_BLOCK * FRAME_BYTES)

/* Pre-allocate several fixed-size TX buffers using k_mem_slab */
#define NUM_BLOCKS         8

#if defined(CONFIG_SOC_NRF54H20_CPUAPP)
#include <dmm.h>
struct k_mem_slab tx_slab;
struct k_mem_slab rx_slab;
char __aligned(WB_UP(4)) mem_slab_tx_buffer[NUM_BLOCKS * WB_UP(BLOCK_SIZE)]
					 DMM_MEMORY_SECTION(DT_ALIAS(i2s_node0));
char __aligned(WB_UP(4)) mem_slab_rx_buffer[NUM_BLOCKS * WB_UP(BLOCK_SIZE)]
					 DMM_MEMORY_SECTION(DT_ALIAS(i2s_node0));                     
#else
K_MEM_SLAB_DEFINE(tx_slab, BLOCK_SIZE, NUM_BLOCKS, 4);
K_MEM_SLAB_DEFINE(rx_slab, BLOCK_SIZE, NUM_BLOCKS, 4);
#endif

static int16_t sine_table[SINE_TABLE_LEN];

/* Generate sine table: -0x7FFF ~ +0x7FFF (avoid clipping) */
#define M_PI 3.14159265358979323846
static void build_sine_table(void)
{
    for (int n = 0; n < SINE_TABLE_LEN; n++) {
        double phase = (2.0 * M_PI * n) / (double)SINE_TABLE_LEN;
        sine_table[n] = (int16_t)(0.9 * 32767.0 * sin(phase)); /* 0.9 FS margin */
    }
}

/* Fill one block with interleaved L/R 400 Hz sine data */
static void fill_block_16bit_stereo(int16_t *dst_lr)
{
    for (int i = 0; i < SINE_TABLE_LEN; i++) {
        int16_t s = sine_table[i];
        *dst_lr++ = s; /* Left */
        *dst_lr++ = s; /* Right (set to 0 for mono output) */
    }
}

int main(void)
{
    int ret;
    uint32_t block_count = 0;
    const struct device *i2s = DEVICE_DT_GET(DT_ALIAS(i2s_node0));

    if (!device_is_ready(i2s)) {
        printk("I2S device not ready\n");
        return -1;
    }

    build_sine_table();

    #if defined(CONFIG_SOC_NRF54H20_CPUAPP)
    ret = k_mem_slab_init(&tx_slab, mem_slab_tx_buffer, WB_UP(BLOCK_SIZE), NUM_BLOCKS);
    if(ret != 0) {
        printk("k_mem_slab_init failed: %d\n", ret);
        return -1;
    }
    ret = k_mem_slab_init(&rx_slab, mem_slab_rx_buffer, WB_UP(BLOCK_SIZE), NUM_BLOCKS);
    if(ret != 0) {
        printk("k_mem_slab_init failed: %d\n", ret);
        return -1;
    }
    #endif

    /* ---------------- I2S TX Configuration ---------------- */
    struct i2s_config tx_cfg = {
        .word_size       = WORD_BITS,
        .channels        = CHANNELS,
        .format          = I2S_FMT_DATA_FORMAT_I2S,
        .options         = I2S_OPT_BIT_CLK_MASTER |
                           I2S_OPT_FRAME_CLK_MASTER, /* SoC provides BCLK/LRCK */
        .frame_clk_freq  = SAMPLE_RATE_HZ,
        .mem_slab        = &tx_slab,
        .block_size      = BLOCK_SIZE,
        .timeout         = SYS_FOREVER_MS,
    };

    /* Enable MCLK if required by the external codec */
    // cfg.options |= I2S_OPT_MCLK_MASTER;

    ret = i2s_configure(i2s, I2S_DIR_TX, &tx_cfg);
    if (ret) {
        printk("i2s_configure failed: %d\n", ret);
        return -1;
    }

        /* ---------------- I2S RX Configuration ---------------- */
    struct i2s_config rx_cfg = {
        .word_size       = WORD_BITS,
        .channels        = CHANNELS,
        .format          = I2S_FMT_DATA_FORMAT_I2S,
        /* RX will use the same clocks from TX (still master on SoC side) */
        .options         = I2S_OPT_BIT_CLK_MASTER |
                           I2S_OPT_FRAME_CLK_MASTER,
        .frame_clk_freq  = SAMPLE_RATE_HZ,
        .mem_slab        = &rx_slab,
        .block_size      = BLOCK_SIZE,
        .timeout         = SYS_FOREVER_MS,
    };

    ret = i2s_configure(i2s, I2S_DIR_RX, &rx_cfg);
    if (ret) {
        printk("i2s_configure RX failed: %d\n", ret);
        return -1;
    }

    /* ---------------- Queue Initial Blocks ---------------- */
    for (int n = 0; n < 2; n++) {
        void *blk = NULL;
        ret = k_mem_slab_alloc(&tx_slab, &blk, K_FOREVER);
        if (ret) {
            printk("slab alloc failed\n");
            return -1;
        }
        fill_block_16bit_stereo((int16_t *)blk);
        ret = i2s_write(i2s, blk, BLOCK_SIZE);
        if (ret) {
            printk("i2s_write failed: %d\n", ret);
            k_mem_slab_free(&tx_slab, &blk);
            return -1;
        }
    }

    printk("I2S 400 Hz tone streaming at 48 kHz...\n");
    
    ret = i2s_trigger(i2s, I2S_DIR_BOTH, I2S_TRIGGER_START);
    if (ret) {
        printk("i2s RX START failed: %d\n", ret);
        return -1;
    }

    /* ---------------- Main Loop: continuously feed blocks ---------------- */
    while (1) {
        void *blk = NULL;

        /* Allocate one block */
        ret = k_mem_slab_alloc(&tx_slab, &blk, K_FOREVER);
        if (ret) {
            printk("slab alloc failed\n");
            break;
        }

        /* Fill data */
        fill_block_16bit_stereo((int16_t *)blk);

        /* Send it to I2S driver */
        ret = i2s_write(i2s, blk, BLOCK_SIZE);
        if (ret) {
            printk("i2s_write failed: %d\n", ret);
            k_mem_slab_free(&tx_slab, &blk);
            break;
        }

        /* The I2S driver will release the buffer after transmission */
        /* ---------- RX path: read one filled block ---------- */
        size_t rx_size = 0U;
        int16_t rx_block[BLOCK_SIZE*2];
        ret = i2s_buf_read(i2s, rx_block, &rx_size);
        if (ret) {
            printk("i2s_read failed: %d\n", ret);
            break;
        }

        if (rx_size != BLOCK_SIZE) {
            printk("Unexpected RX block size: %zu (expected %d)\n",
                   rx_size, BLOCK_SIZE);
        } else {
            /* Inspect RX samples (do not print too often) */
            if ((block_count % 1000) == 0) {
                printk("RX block %u: L0=%d, R0=%d L1=%d, R1=%d\n",
                       block_count, rx_block[0], rx_block[1], rx_block[2], rx_block[3]);
            }
        }
        block_count++;
    }

    /* ---------------- Stop TX on exit or error ---------------- */
    (void)i2s_trigger(i2s, I2S_DIR_TX, I2S_TRIGGER_STOP);
    (void)i2s_trigger(i2s, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
    (void)i2s_trigger(i2s, I2S_DIR_TX, I2S_TRIGGER_PREPARE);
	return -1;
}
