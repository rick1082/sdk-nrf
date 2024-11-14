/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bt_mgmt_ctlr_cfg_internal.h"

#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/logging/log_ctrl.h>

#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_mgmt_ctlr_cfg, CONFIG_BT_MGMT_CTLR_CFG_LOG_LEVEL);

#define COMPANY_ID_NORDIC 0x0059

#define WDT_TIMEOUT_MS	      3000
#define CTLR_POLL_INTERVAL_MS (WDT_TIMEOUT_MS - 1000)

#define CTLR_POLL_WORK_STACK_SIZE 1024

K_THREAD_STACK_DEFINE(ctlr_poll_stack_area, CTLR_POLL_WORK_STACK_SIZE);


int bt_mgmt_ctlr_cfg_manufacturer_get(bool print_version, uint16_t *manufacturer)
{
	int ret;
	struct net_buf *rsp;

	ret = bt_hci_cmd_send_sync(BT_HCI_OP_READ_LOCAL_VERSION_INFO, NULL, &rsp);
	if (ret) {
		return ret;
	}

	struct bt_hci_rp_read_local_version_info *rp = (void *)rsp->data;

	if (print_version) {
		if (rp->manufacturer == COMPANY_ID_NORDIC) {
			/* NOTE: The string below is used by the Nordic CI system */
			LOG_INF("Controller: SoftDevice: Version %s (0x%02x), Revision %d",
				bt_hci_get_ver_str(rp->hci_version), rp->hci_version,
				rp->hci_revision);
		} else {
			LOG_ERR("Unsupported controller");
			return -EPERM;
		}
	}

	*manufacturer = sys_le16_to_cpu(rp->manufacturer);

	net_buf_unref(rsp);

	return 0;
}

int bt_mgmt_ctlr_cfg_init(bool watchdog_enable)
{
	int ret;
	uint16_t manufacturer = 0;

	ret = bt_mgmt_ctlr_cfg_manufacturer_get(true, &manufacturer);
	if (ret) {
		return ret;
	}

	return 0;
}
