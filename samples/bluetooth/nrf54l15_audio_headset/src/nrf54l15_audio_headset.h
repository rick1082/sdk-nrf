/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef NRF54L15_AUDIO_HEADSET_H_
#define NRF54L15_AUDIO_HEADSET_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/bluetooth/assigned_numbers.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/net_buf.h>
#include <zephyr/types.h>

#define SINK_CONTEXT BT_AUDIO_CONTEXT_TYPE_ANY
#define SOURCE_CONTEXT                                                                             \
	(BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL |                \
	 BT_AUDIO_CONTEXT_TYPE_GAME | BT_AUDIO_CONTEXT_TYPE_VOICE_ASSISTANTS |                     \
	 BT_AUDIO_CONTEXT_TYPE_LIVE | BT_AUDIO_CONTEXT_TYPE_INSTRUCTIONAL)

/* Log every Nth received SDU (plus the first one and any lost/erroneous ones) */
#define ISO_RX_LOG_INTERVAL 100U

/** Statistics for a receiving (CIS or BIS) stream */
struct iso_rx_stats {
	size_t recv_cnt;
	size_t valid_cnt;
	size_t empty_cnt;
	size_t lost_cnt;
	size_t err_cnt;
};

/**
 * @brief Initialize the VCP Volume Renderer role
 *
 * @return 0 if success, errno on failure.
 */
int vcp_vol_renderer_init(void);

/**
 * @brief Initialize BAP Unicast Server (CIS sink and source)
 *
 * @return 0 if success, errno on failure.
 */
int bap_unicast_sr_init(void);

/**
 * @brief Initialize BAP Broadcast Sink and Scan Delegator (BIS)
 *
 * @return 0 if success, errno on failure.
 */
int bap_broadcast_sink_init(void);

/**
 * @brief Reset the receive statistics of a stream
 *
 * @param stats Statistics to reset
 */
void iso_rx_reset(struct iso_rx_stats *stats);

/**
 * @brief Update statistics for a received ISO SDU and print it when relevant
 *
 * @param tag    Short tag identifying the transport, e.g. "CIS" or "BIS"
 * @param stream The stream the SDU was received on
 * @param stats  Statistics of @p stream
 * @param info   ISO receive info
 * @param buf    The received SDU
 */
void iso_rx_log(const char *tag, const struct bt_bap_stream *stream, struct iso_rx_stats *stats,
		const struct bt_iso_recv_info *info, const struct net_buf *buf);

/** Source of the audio played by the audio output */
enum audio_out_src {
	AUDIO_OUT_SRC_CIS,
	AUDIO_OUT_SRC_BIS,
};

/**
 * @brief Initialize the audio output (TLV320 DAC and I2S)
 *
 * Must be called before Bluetooth is enabled.
 *
 * @return 0 if success, errno on failure.
 */
int audio_out_init(void);

/**
 * @brief Start playing a stream
 *
 * Up to two streams (left and right) from the same source are played at a time. The first
 * stream sets the codec configuration used for decoding.
 *
 * @param src    Source of the stream
 * @param stream The stream that started
 *
 * @retval 0 if the stream is played
 * @retval -EBUSY if streams from another source are being played
 * @retval -ENOMEM if two streams are already being played
 * @retval -EINVAL if the codec configuration of the stream is not supported
 */
int audio_out_stream_start(enum audio_out_src src, const struct bt_bap_stream *stream);

/**
 * @brief Stop playing a stream
 *
 * Does nothing if the stream is not being played.
 *
 * @param stream The stream that stopped
 */
void audio_out_stream_stop(const struct bt_bap_stream *stream);

/**
 * @brief Play an SDU received on a stream
 *
 * Does nothing if the stream is not being played.
 *
 * @param stream The stream the SDU was received on
 * @param info   ISO receive info
 * @param buf    The received SDU
 */
void audio_out_recv(const struct bt_bap_stream *stream, const struct bt_iso_recv_info *info,
		    const struct net_buf *buf);

/**
 * @brief Set the output volume
 *
 * @param volume Volume from 0 to 255, as used by the Volume Control Service
 * @param mute   True to mute the output
 */
void audio_out_volume_set(uint8_t volume, bool mute);

#endif /* NRF54L15_AUDIO_HEADSET_H_ */
