/*
 * Copyright (c) 2024 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cmsis_core.h>

#ifdef CONFIG_PLATFORM_SPECIFIC_INIT
void z_arm_platform_init(void)
{
	/* Needs to implement SOC specific SystemInit API */
}
#endif /* CONFIG_PLATFORM_SPECIFIC_INIT */
