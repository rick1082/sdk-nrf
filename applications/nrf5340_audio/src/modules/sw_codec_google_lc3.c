/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: Nordic-5-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be
 * reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <zephyr/kernel.h>
#include "lc3.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sw_codec_google_lc3);

static lc3_encoder_mem_48k_t lc3_encoder_mem[2];
static lc3_decoder_mem_48k_t lc3_decoder_mem[2];
static lc3_encoder_t enc_handle_ch[2];
static lc3_decoder_t dec_handle_ch[2];
#define LC3_USE_BITRATE_FROM_INIT 0
static uint32_t m_enc_bitrate;
static uint16_t m_framesize_us;
static uint16_t m_pcm_sample_rate;

int sw_codec_lc3_enc_run(void const *const pcm_data, uint32_t pcm_data_size, uint32_t enc_bitrate,
			 uint8_t audio_ch, uint16_t lc3_data_buf_size, uint8_t *const lc3_data,
			 uint16_t *const lc3_data_wr_size)
{
	int ret;

	int target_bytes = lc3_frame_bytes(m_framesize_us, m_enc_bitrate);
	
	ret = lc3_encode(enc_handle_ch[audio_ch], LC3_PCM_FORMAT_S16, pcm_data, 1, target_bytes, lc3_data);
	if (ret) {
		LOG_INF("ret = %d, LC3 target_bytes = %d, enc_bitrate = %d, audio_ch =%d", ret, target_bytes, m_enc_bitrate, audio_ch);
	}

	*lc3_data_wr_size = target_bytes;

	return 0;
}

int sw_codec_lc3_dec_run(uint8_t const *const lc3_data, uint16_t lc3_data_size,
			 uint16_t pcm_data_buf_size, uint8_t audio_ch, void *const pcm_data,
			 uint16_t *const pcm_data_wr_size, bool bad_frame)
{
	int ret;

	if (!dec_handle_ch[audio_ch]) {
		LOG_ERR("LC3 dec ch:%d is not initialized", audio_ch);
		return -EPERM;
	}

	if (bad_frame) {
		ret = lc3_decode(dec_handle_ch[audio_ch], NULL, lc3_data_size, LC3_PCM_FORMAT_S16, pcm_data, 1);
	} else {
		ret = lc3_decode(dec_handle_ch[audio_ch], lc3_data, lc3_data_size, LC3_PCM_FORMAT_S16, pcm_data, 1);
	}

	if (ret < 0) {
		LOG_ERR("Failed to decode");
		return -EINVAL;
	}

	*pcm_data_wr_size = lc3_frame_samples(m_framesize_us, m_pcm_sample_rate) * 2;

	return 0;
}

int sw_codec_lc3_enc_uninit_all(void)
{
	return 0;
}

int sw_codec_lc3_dec_uninit_all(void)
{
	return 0;
}

int sw_codec_lc3_init(uint8_t *sw_codec_lc3_buffer, uint32_t *sw_codec_lc3_buffer_size,
		      uint16_t framesize_us)
{
	return 0;
}

int sw_codec_lc3_enc_init(uint16_t pcm_sample_rate, uint8_t pcm_bit_depth, uint16_t framesize_us,
			  uint32_t enc_bitrate, uint8_t num_channels, uint16_t *const pcm_bytes_req)
{
	m_framesize_us = framesize_us;
	m_enc_bitrate = enc_bitrate;
	m_pcm_sample_rate = pcm_sample_rate;
	for (int i = 0; i < num_channels; i++) {
		if (enc_handle_ch[i] == NULL) {
			enc_handle_ch[i] = lc3_setup_encoder(framesize_us, pcm_sample_rate, 0,
								&lc3_encoder_mem[i]);
		} else {
			LOG_WRN("LC3 encoder handle [%d] already initializedd", i);
		}
	}

	return 0;
}

int sw_codec_lc3_dec_init(uint16_t pcm_sample_rate, uint8_t pcm_bit_depth, uint16_t framesize_us,
			  uint8_t num_channels)
{
	m_framesize_us = framesize_us;
	m_pcm_sample_rate = pcm_sample_rate;

	for (int i = 0; i < num_channels; i++) {
		if (dec_handle_ch[i] == NULL) {
			dec_handle_ch[i] = lc3_setup_decoder(framesize_us, pcm_sample_rate, 0, &lc3_decoder_mem[i]);
			//LOG_INF("LC3 decoder handle [%d] initialized", i);
		} else {
			LOG_WRN("LC3 decoder handle [%d] already initialized", i);
		}
	}

	return 0;
}
