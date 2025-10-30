#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <zephyr/sys/ring_buffer.h>
#include <math.h>
#include <dk_buttons_and_leds.h>

#include "ml_main.h"

#define SAMPLE_LENGTH_IN_MS 100
#define PDM_BYTE_DEPTH 2
#define PDM_SAMPLE_PER_SEC 16000

extern void cpu_blocks_init(void);
extern void mfcc_cpu(const int16_t* audio_sample, int8_t* mfcc_result);
extern void inference_cpu(const int8_t* mfcc_feature_map, int* class_index);



#define MFCC_STEP_IN_MS 20
#define MFCC_WINDOW_IN_MS 30

/*
 * Local buffer to copy data from PDM buffer. Extra space for data corrsponding one MFCC window
 * is added to hold residual part
 */

static int16_t mfcc_input[PDM_SAMPLE_PER_SEC*SAMPLE_LENGTH_IN_MS/MSEC_PER_SEC+PDM_SAMPLE_PER_SEC*MFCC_WINDOW_IN_MS/MSEC_PER_SEC];


#define MFCC_RING_BUF_BYTES 1000
RING_BUF_DECLARE(mfcc_int8_ring_buf, MFCC_RING_BUF_BYTES);

/* Semephore to sync mfcc calculation and kws inference. */
/* It's binary as we let the inference workflow to determine how many rounds of inference should happen */
K_SEM_DEFINE(mfcc_ready, 0 ,1);

#define MFCC_DATA_LENGTH 10
#define KWS_MFCC_WINDOW 49
#define KWS_MFCC_STEP 10
int8_t inference_input[KWS_MFCC_WINDOW*MFCC_DATA_LENGTH];

const char* label[] = {"Down", "Go", "Left", "No", "Off", "On", "Right", "Stop", "Up", "Yes", "Silence", "Unknown"};

#define INFERENCE_STACK_SIZE 4096
#define INFERENCE_PRIORITY 5

void inference_thread(void *, void *, void *) {
#define FILTER_WINDOW_LEN 4
	int last_n_results[FILTER_WINDOW_LEN];
	for (int i = 0; i < FILTER_WINDOW_LEN; i++) {
				last_n_results[i] = -1;
	}
	while (true) {

		k_sem_take(&mfcc_ready, K_FOREVER);
		while (ring_buf_size_get(&mfcc_int8_ring_buf) >= MFCC_DATA_LENGTH * KWS_MFCC_WINDOW) {
			int max_idx = 0;
			ring_buf_peek(&mfcc_int8_ring_buf, (uint8_t*)inference_input, MFCC_DATA_LENGTH * KWS_MFCC_WINDOW);


			inference_cpu(inference_input, &max_idx);



			// printk("[%u] Label: %s, likelihood %f\n",end, label[max_idx],
			// 	output_scale * (output->data.int8[max_idx] - output_zp));
			for (int i = 0; i < FILTER_WINDOW_LEN - 1; i++) {
				last_n_results[i] = last_n_results[i+1];
			}
			last_n_results[FILTER_WINDOW_LEN - 1] = max_idx;
			bool all_same = true;
			for (int i = 0; i < FILTER_WINDOW_LEN - 1; i++) {
				if (last_n_results[i] != last_n_results[i+1]) {
					all_same = false;
					break;
				}
			}
			if (all_same == true) {
				printk("Label: %s in last %d inferences.\n", label[max_idx], FILTER_WINDOW_LEN);
				if (max_idx == 4) {
					dk_set_led_off(DK_LED4);
				}
				if (max_idx == 5) {
					dk_set_led_on(DK_LED4);
				}
				for (int i = 0; i < FILTER_WINDOW_LEN; i++) {
					last_n_results[i] = -1;
				}
			}

			/* How many data we remove determines how frequency the inference is */
			/* Every step length of 1 equal to 20 ms */
			ring_buf_get(&mfcc_int8_ring_buf, NULL, KWS_MFCC_STEP*MFCC_DATA_LENGTH);

		}

	}

}

K_THREAD_STACK_DEFINE(inference_stack_area, INFERENCE_STACK_SIZE);
struct k_thread inference_thread_data;

uint32_t mfcc_input_tail;

int ml_init(void) {
	cpu_blocks_init();

	/*start inference thread*/
	k_tid_t my_tid = k_thread_create(&inference_thread_data, inference_stack_area,
                                 K_THREAD_STACK_SIZEOF(inference_stack_area),
                                 inference_thread,
                                 NULL, NULL, NULL,
                                 INFERENCE_PRIORITY, 0, K_NO_WAIT);
	printk("inference thread id %d\n", my_tid);

	mfcc_input_tail = 0;
	return 0;
}
int ml_process(void* buffer, size_t size) {


	int sample_len;

	sample_len = size / 2;

	for (int i = 0; i < sample_len; i++) {
		mfcc_input[mfcc_input_tail] = ((int16_t*)buffer)[i];
		mfcc_input_tail++;
	}

	uint32_t start, end;
	// Calculate MFCC feature map
	start = k_uptime_get_32();

	int number_of_windows = (mfcc_input_tail - (PDM_SAMPLE_PER_SEC*MFCC_WINDOW_IN_MS/MSEC_PER_SEC)
		+ (PDM_SAMPLE_PER_SEC*MFCC_STEP_IN_MS/MSEC_PER_SEC)) / (PDM_SAMPLE_PER_SEC*MFCC_STEP_IN_MS/MSEC_PER_SEC);
		for (int i = 0; i < number_of_windows; i++) {

		int8_t mfcc_slice_int8[MFCC_DATA_LENGTH];

		mfcc_cpu(&(mfcc_input[PDM_SAMPLE_PER_SEC*MFCC_STEP_IN_MS/MSEC_PER_SEC*i]), mfcc_slice_int8);



		uint32_t ret;

		ret = ring_buf_put(&mfcc_int8_ring_buf, (const uint8_t*)mfcc_slice_int8, sizeof(mfcc_slice_int8));
		if (ret != sizeof(mfcc_slice_int8)) {
			printk("Ring buffer full!, queued %d, expected 10\n", ret);
			/* The return value is not checked because there should be enough data to discard */
			// ring_buf_get(&mfcc_int8_ring_buf, NULL, 10 - ret);
			// ring_buf_put(&mfcc_int8_ring_buf, (const uint8_t*)&(mfcc_slice_int8[ret]), 10 - ret);
		}
	}
	end = k_uptime_get_32();
	// printk("Time used for %d mfcc calculation: %d ms.\n", number_of_windows, end - start);
	for (uint32_t i = 0; i < mfcc_input_tail - PDM_SAMPLE_PER_SEC*MFCC_STEP_IN_MS/MSEC_PER_SEC * number_of_windows; i++) {
		mfcc_input[i] = mfcc_input[PDM_SAMPLE_PER_SEC*MFCC_STEP_IN_MS/MSEC_PER_SEC * number_of_windows + i];
	}
	mfcc_input_tail = mfcc_input_tail - PDM_SAMPLE_PER_SEC*MFCC_STEP_IN_MS/MSEC_PER_SEC * number_of_windows;

	/* Trigger inference here */
	if (ring_buf_size_get(&mfcc_int8_ring_buf) >= MFCC_DATA_LENGTH * KWS_MFCC_WINDOW) {
		k_sem_give(&mfcc_ready);
	}
	return 0;
}