# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf54l15_audio_headset)

target_sources(app PRIVATE
  src/main.c
  src/vcp_vol_renderer.c
  src/bap_broadcast_sink.c
  src/bap_unicast_sr.c
  src/iso_rx.c
  src/audio_out.c
)
