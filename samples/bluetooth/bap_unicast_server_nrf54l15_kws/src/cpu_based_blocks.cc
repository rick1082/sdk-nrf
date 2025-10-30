#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include <dsp/transform_functions.h>
#include <dsp/statistics_functions.h>
#include <dsp/basic_math_functions.h>
#include <dsp/complex_math_functions.h>
#include <dsp/fast_math_functions.h>
#include <dsp/matrix_functions.h>
#include "mfcc_data.h"

#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/micro/micro_log.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/system_setup.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include "kws_model_data.h"

#include <zephyr/logging/log.h>
#define LOG_LEVEL 2
LOG_MODULE_DECLARE(dmic_sample);


#include <math.h>

static arm_mfcc_instance_f32 S_mfcc;
static float cmsis_mfcc_input[512]; // hold mfcc input
float cmsis_mfcc_tmp[1024]; // tmp buffer to mfcc

const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *input = nullptr;
TfLiteTensor *output = nullptr;
float input_scale, output_scale;
int input_zp, output_zp;
constexpr int kTensorArenaSize = 30000;
uint8_t tensor_arena[kTensorArenaSize];



inline void arm_mfcc_f32_modified(
	const arm_mfcc_instance_f32 * S,
	float32_t *pSrc,
	float32_t *pDst,
	float32_t *pTmp
	)
{

	// float32_t maxValue;
	// uint32_t  index;
	uint32_t i;
	float32_t result;
	const float32_t *coefs=S->filterCoefs;
	arm_matrix_instance_f32 pDctMat;

	/* Normalize */
	// arm_absmax_f32(pSrc,S->fftLen,&maxValue,&index);

	// if (maxValue != 0.0f)
	// {
	//    arm_scale_f32(pSrc,1.0f/maxValue,pSrc,S->fftLen);
	// }

	/* Multiply by window */
	arm_mult_f32(pSrc,S->windowCoefs,pSrc,S->fftLen);

	/* Compute spectrum magnitude
	*/
#if defined(ARM_MFCC_CFFT_BASED)
	/* some HW accelerator for CMSIS-DSP used in some boards
		are only providing acceleration for CFFT.
		With ARM_MFCC_CFFT_BASED enabled, CFFT is used and the MFCC
		will be accelerated on those boards.

		The default is to use RFFT
	*/
	/* Convert from real to complex */
	for(i=0; i < S->fftLen ; i++)
	{
		pTmp[2*i] = pSrc[i];
		pTmp[2*i+1] = 0.0f;
	}
	arm_cfft_f32(&(S->cfft),pTmp,0,1);
#else
	/* Default RFFT based implementation */
	arm_rfft_fast_f32(&(S->rfft),pSrc,pTmp,0);
	/* Unpack real values */
	pTmp[S->fftLen]=pTmp[1];
	pTmp[S->fftLen+1]=0.0f;
	pTmp[1]=0.0f;
#endif
	// Shouldn't the length be S->fftLen + 2 (=514) or 257 complex points ?
	arm_cmplx_mag_f32(pTmp,pSrc,(S->fftLen + 2)/2);
	// if (maxValue != 0.0f)
	// {
	//    arm_scale_f32(pSrc,maxValue,pSrc,S->fftLen);
	// }

	/* Apply MEL filters */
	for(i=0; i<S->nbMelFilters; i++)
	{
		arm_dot_prod_f32(pSrc+S->filterPos[i],
			coefs,
			S->filterLengths[i],
			&result);

		coefs += S->filterLengths[i];

		pTmp[i] = result;

	}

	/* Compute the log */
	arm_offset_f32(pTmp,1.0e-6f,pTmp,S->nbMelFilters);
	arm_vlog_f32(pTmp,pTmp,S->nbMelFilters);

	/* Multiply with the DCT matrix */
	pDctMat.numRows=S->nbDctOutputs;
	pDctMat.numCols=S->nbMelFilters;
	pDctMat.pData=(float32_t*)S->dctCoefs;

	arm_mat_vec_mult_f32(&pDctMat, pTmp, pDst);
}




int cpu_blocks_init(void) {
	uint32_t start, end;


	arm_mfcc_init_f32(&S_mfcc,
		512,
		40,
		10,
		mfcc_dct_coefs_config1_f32,
		mfcc_filter_pos_config1_f32,
		mfcc_filter_len_config1_f32,
		mfcc_filter_coefs_config1_f32,
		mfcc_window_coefs_config1_f32);

	model = tflite::GetModel(kws_model_data);
	if (model->version() != TFLITE_SCHEMA_VERSION) {
		MicroPrintf("Model provided is schema version %d not equal "
					"to supported version %d.",
					model->version(), TFLITE_SCHEMA_VERSION);
		return 1;
	}
	static tflite::MicroMutableOpResolver <6> micro_op_resolver;
	micro_op_resolver.AddFullyConnected();
    micro_op_resolver.AddAveragePool2D();
    micro_op_resolver.AddConv2D();
    micro_op_resolver.AddDepthwiseConv2D();
    micro_op_resolver.AddReshape();
    micro_op_resolver.AddSoftmax();

	static tflite::MicroInterpreter static_interpreter(
		model, micro_op_resolver, tensor_arena, kTensorArenaSize);
	interpreter = &static_interpreter;

	/* Allocate memory from the tensor_arena for the model's tensors. */
	TfLiteStatus allocate_status = interpreter->AllocateTensors();
	if (allocate_status != kTfLiteOk) {
		MicroPrintf("AllocateTensors() failed");
		return 1;
	}

	/* Obtain pointers to the model's input and output tensors. */
	input = interpreter->input(0);
	output = interpreter->output(0);



	input_scale = ((TfLiteAffineQuantization*)input->quantization.params)->scale->data[0];
	input_zp = ((TfLiteAffineQuantization*)input->quantization.params)->zero_point->data[0];

	output_scale = ((TfLiteAffineQuantization*)output->quantization.params)->scale->data[0];
	output_zp = ((TfLiteAffineQuantization*)output->quantization.params)->zero_point->data[0];

	printk("input tensor quant spec: zero point = %d, scale = %f\n", input_zp, input_scale);
	printk("output tensor quant spec: zero point = %d, scale = %f\n", output_zp, output_scale);

	return 0;
}

void mfcc_cpu(const int16_t* audio_sample, int8_t* mfcc_result) {
    float mfcc_slice[10];

	for (int i = 0; i < 480; i++) {
		cmsis_mfcc_input[i] = audio_sample[i] / 32768.0f;
	}
	arm_mfcc_f32_modified(&S_mfcc, cmsis_mfcc_input, mfcc_slice, cmsis_mfcc_tmp);

	/* Quantize based on input tensor spec */
	for (int j = 0; j < 10; j++) {
		int32_t tmp;
		// tmp = static_cast<int32_t>(mfcc_slice[j] / input_scale) + input_zp;
		tmp = static_cast<int32_t>(round(mfcc_slice[j] / input_scale)) + input_zp;
		if (tmp > INT8_MAX) {
			mfcc_result[j] = INT8_MAX;
			LOG_INF("too large! %d\n", tmp);
		} else if (tmp < INT8_MIN) {
			mfcc_result[j] = INT8_MIN;
			LOG_INF("too small! %d\n", tmp);
		} else {
			mfcc_result[j] = static_cast<int8_t>(tmp);
		}
	}

}

void inference_cpu(const int8_t* mfcc_feature_map, int* class_index) {

	uint32_t start, end;
	start = k_uptime_get_32();
	TfLiteStatus invoke_status = kTfLiteOk;

	for (int i = 0; i < 490; i++) {
		(input->data.int8)[i] = mfcc_feature_map[i];
	}

	invoke_status = interpreter->Invoke();
	end = k_uptime_get_32();
	// printk("Time used for inference: %d ms.\n", end - start);
	if (invoke_status != kTfLiteOk) {
		printk("Invoke failed\n");
		return;
	}

	/* Retrieve result */
	int max_idx = 0;
	for (int i = 0; i < 12; i++) {
		if (output->data.int8[max_idx] < output->data.int8[i]) {
			max_idx = i;
		}
	}
	*class_index = max_idx;

	return;
}