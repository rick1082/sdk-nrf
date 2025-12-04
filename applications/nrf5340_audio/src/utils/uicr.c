/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "uicr.h"

#include <stdint.h>
#include <errno.h>
//#include <nrfx_nvmc.h>

/* Memory address to store segger number of the board */
#define MEM_ADDR_UICR_SNR UICR_APP_BASE_ADDR
/* Memory address to store the location intended to be used for this board */
#define MEM_ADDR_UICR_CH (MEM_ADDR_UICR_SNR + sizeof(uint32_t))

uint32_t uicr_location_get(void)
{
	return 0;//*(uint32_t *)MEM_ADDR_UICR_CH;
}

int uicr_location_set(uint32_t channel)
{
	return 0;
}

uint64_t uicr_snr_get(void)
{
	return 0;
}
