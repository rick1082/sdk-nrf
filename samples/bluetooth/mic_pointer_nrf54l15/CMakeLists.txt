# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mic_pointer_nrf54l15)

target_sources(app PRIVATE
  src/main.c
  src/le_audio_mic.c
)
