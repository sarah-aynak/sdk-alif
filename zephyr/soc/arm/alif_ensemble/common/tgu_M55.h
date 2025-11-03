/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TGU_M55_H_
#define TGU_M55_H_
#include <zephyr/devicetree.h>
#include <stdint.h>
#include <limits.h>

#define ALIF_TGU_LUT(base, n) ((base) + 0x10 + 4 * (n))

#define ALIF_TGU_SET_BIT_RANGE(from, to)                                      \
	(~0U >> ((CHAR_BIT * sizeof(unsigned int)) - (to)-1)) & (~0U << (from))

/* NS REGIONs, should be 128kb aligned */
#if DT_NODE_EXISTS(DT_NODELABEL(ns))
#define NS_REGION_0_BASE DT_REG_ADDR(DT_NODELABEL(ns))
#define NS_REGION_0_SIZE DT_REG_SIZE(DT_NODELABEL(ns))
#define NS_REGION_0_END  (NS_REGION_0_BASE + NS_REGION_0_SIZE) /* 384kb */
#else
#error "Non-secure DTS node not found."
#endif /* DT_NODE_EXISTS(DT_NODELABEL(ns)) */

enum tcm_type {
	TCM_TYPE_ITCM,
	TCM_TYPE_DTCM
};

struct mem_region {
	uint32_t start;
	uint32_t end;
	enum tcm_type type;
};

/*
 * alif_tgu_setup()
 * Set up the TGU look up tables as per the information provided by
 * the platform in ns_regions array.
 */
void alif_tgu_setup(void);
#endif /* TGU_M55_H_ */
