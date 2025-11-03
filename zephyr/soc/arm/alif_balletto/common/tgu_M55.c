/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/common/sys_io.h>
#include <cmsis_core.h>
#include "tgu_M55.h"

#define ITCM_BASE DT_REG_ADDR(DT_NODELABEL(itcm))
#define DTCM_BASE DT_REG_ADDR(DT_NODELABEL(dtcm))

/*
 * Array of NS memory regions. Add new regions to this list.
 */
static const struct mem_region ns_regions[] = {
	{NS_REGION_0_BASE, NS_REGION_0_END, TCM_TYPE_DTCM},
};

/*
 * TGU setup function. Marks single memory region starting at NS_REGION_1_BASE
 * and ending at NS_REGION_1_END as Non Secure in the TGU look up tables. The
 * region should be in the DTCM area. Can be extended to include the ITCM region
 * and multiple discontigous regions if needed.
 */
void alif_tgu_setup(void)
{
	uint32_t start_block, end_block, start_lut, end_lut;
	uint32_t start_offset, end_offset, lut_val_l, lut_val_h;
	uint32_t base, itcm_blksize, dtcm_blksize, blksize;
	uint32_t i;

	/* Find out the TGU block size for ITCM */
	blksize = MEMSYSCTL->ITGU_CFG & MEMSYSCTL_ITGU_CFG_BLKSZ_Msk;
	/* ITCM blksize is 2^(blksize + 5)*/
	itcm_blksize = 1 << (blksize + 5);

	/* Find out the TGU block size for DTCM */
	blksize = MEMSYSCTL->DTGU_CFG & MEMSYSCTL_DTGU_CFG_BLKSZ_Msk;
	/* DTCM block size is 2^(blksize + 5) */
	dtcm_blksize = 1 << (blksize + 5);

	for (i = 0; i < ARRAY_SIZE(ns_regions); i++) {
		if (ns_regions[i].type == TCM_TYPE_DTCM) {
			start_block = (ns_regions[i].start - DTCM_BASE) / dtcm_blksize;
			end_block = (ns_regions[i].end - DTCM_BASE) / dtcm_blksize;
			base = (uint32_t)&MEMSYSCTL->DTGU_CTRL;
		} else {
			start_block = (ns_regions[i].start - ITCM_BASE) / itcm_blksize;
			end_block = (ns_regions[i].end - ITCM_BASE) / itcm_blksize;
			base = (uint32_t)&MEMSYSCTL->ITGU_CTRL;
		}

		start_lut = start_block / 32;
		end_lut = end_block / 32;

		start_offset = start_block % 32;
		end_offset = end_block % 32;

		if (start_lut == end_lut) {
			/* same LUT register */
			lut_val_l = ALIF_TGU_SET_BIT_RANGE(start_offset, end_offset);
			*((volatile uint32_t *)ALIF_TGU_LUT(base, start_lut)) |= lut_val_l;
		} else {
			/* the range spans multiple LUT registers */
			lut_val_l = ALIF_TGU_SET_BIT_RANGE(start_offset, 31);
			lut_val_h = ALIF_TGU_SET_BIT_RANGE(0, end_offset);

			/* Write into the first LUT register */
			*((volatile uint32_t *)ALIF_TGU_LUT(base, start_lut)) = lut_val_l;

			/* Now write to all the intermediate LUT registers */
			while (start_lut != (end_lut - 1)) {
				start_lut++;
				*((volatile uint32_t *)ALIF_TGU_LUT(base, start_lut)) = 0xFFFFFFFFU;
			}

			/* Write into the last LUT register*/
			*((volatile uint32_t *)ALIF_TGU_LUT(base, end_lut)) |= lut_val_h;
		}
	}
}
