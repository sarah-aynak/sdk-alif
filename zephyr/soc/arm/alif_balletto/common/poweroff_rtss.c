/*
 * Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <cmsis_core.h>
#include <pm_rtss.h>

void z_sys_poweroff(void)
{
	__disable_irq();
	__set_BASEPRI(0);
	pm_core_enter_deep_sleep_request_subsys_off();

	CODE_UNREACHABLE;
}
