/*
 * Copyright (c) 2021-2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LE_AUDIO_MIC_H_
#define LE_AUDIO_MIC_H_

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/bluetooth/uuid.h>

/*
 * The board has a microphone and no speaker, so it is a source only. An
 * available sink context of 0x0000 tells the client there is nothing to render
 * to, which keeps hosts (Windows in particular) from creating an output
 * endpoint for the device.
 */
#define LE_AUDIO_MIC_SINK_CONTEXT   0x0000
#define LE_AUDIO_MIC_SOURCE_CONTEXT BT_AUDIO_CONTEXT_TYPE_ANY

/*
 * BAP unicast announcement, for use inside BT_DATA_BYTES(BT_DATA_SVC_DATA16, ...).
 * This is what makes a unicast client recognise the device as an audio endpoint.
 */
#define LE_AUDIO_MIC_ADV_SVC_DATA                                                                  \
	BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL),                                                       \
	BT_AUDIO_UNICAST_ANNOUNCEMENT_TARGETED,                                                    \
	BT_BYTES_LIST_LE16(LE_AUDIO_MIC_SINK_CONTEXT),                                             \
	BT_BYTES_LIST_LE16(LE_AUDIO_MIC_SOURCE_CONTEXT),                                           \
	0x00 /* Metadata length */

/**
 * @brief Start the HF clock and bring up the PDM microphone.
 *
 * Must be called before bt_enable().
 */
int le_audio_mic_init(void);

/**
 * @brief Register the unicast server, PACS capabilities and audio streams.
 *
 * Must be called after bt_enable().
 */
int le_audio_mic_start(void);

#endif /* LE_AUDIO_MIC_H_ */
