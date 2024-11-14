/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Button assignments
 *
 * Button mappings are listed here.
 *
 */

#ifndef _BUTTON_ASSIGNMENTS_H_
#define _BUTTON_ASSIGNMENTS_H_

#include <zephyr/drivers/gpio.h>

/** @brief List of buttons and associated metadata
 */
enum button_pin_names {
	BUTTON_VOLUME_DOWN,
	BUTTON_VOLUME_UP,
	BUTTON_PLAY_PAUSE,
	BUTTON_4,
	BUTTON_5,
};

#endif /* _BUTTON_ASSIGNMENTS_H_ */
