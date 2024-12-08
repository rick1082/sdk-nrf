#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_sources(app PRIVATE
	       ${CMAKE_CURRENT_SOURCE_DIR}/channel_assignment.c
	       ${CMAKE_CURRENT_SOURCE_DIR}/error_handler.c
	       ${CMAKE_CURRENT_SOURCE_DIR}/uicr.c
)

target_sources_ifdef(CONFIG_BOARD_NRF5340_AUDIO_DK_NRF5340_CPUAPP app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/nrf5340_audio_dk.c
		${CMAKE_CURRENT_SOURCE_DIR}/board_version.c)

zephyr_link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/chmap_filter/lib/cortex-m4/hard-float/libchmapfilt.a)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/chmap_filter/include)