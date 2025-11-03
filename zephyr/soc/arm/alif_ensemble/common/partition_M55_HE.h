/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PARTITION_M55_HE_H_
#define PARTITION_M55_HE_H_

#include <tgu_M55.h>

/*
 * Initialize Security Attribution Unit (SAU) CTRL register
 */
#define SAU_INIT_CTRL 1

/*
 * Enable SAU
 */
#define SAU_INIT_CTRL_ENABLE 1

/*
 * When SAU is disabled
 *   0-> All Memory is Secure
 *   1-> All Memory is Non-Secure
 * Value for SAU->CTRL register bit ALLNS
 * When all Memory is Non-Secure (ALLNS is 1), IDAU can override memory map configuration.
 */
#define SAU_INIT_CTRL_ALLNS 0

/*
 * Initialize Security Attribution Unit (SAU) Address Regions
 * SAU configuration specifies regions to be one of:
 *    - Secure and Non-Secure Callable
 *    - Non-Secure
 * Note: All memory regions not configured by SAU are Secure
 */
#define SAU_REGIONS_MAX 8 /* Max. number of SAU regions */

/*
 * Initialize SAU Region 0
 * Setup SAU Region 0 memory attributes
 */
#define SAU_INIT_REGION0 1

/*
 * Start Address <0-0xFFFFFFE0>
 */
#define SAU_INIT_START0 NS_REGION_0_BASE /* start address of SAU region 0 */

/*
 * End Address <0x1F-0xFFFFFFFF>
 */
#define SAU_INIT_END0 NS_REGION_0_END /* end address of SAU region 0 */

/*
 * Region is
 *  0->Non-Secure
 *  1->Secure, Non-Secure Callable
 */
#define SAU_INIT_NSC0 0

/*
 * Initialize SAU Region 1
 * Setup SAU Region 1 memory attributes
 */
#define SAU_INIT_REGION1 0

/*
 * Start Address <0-0xFFFFFFE0>
 */
#define SAU_INIT_START1 0x00000000

/*
 * End Address <0x1F-0xFFFFFFFF>
 */
#define SAU_INIT_END1 0x00000000

/*
 * Region is
 *  0->Non-Secure
 *  1->Secure, Non-Secure Callable
 */
#define SAU_INIT_NSC1 0

/*
 * Initialize SAU Region 2
 * Setup SAU Region 2 memory attributes
 */
#define SAU_INIT_REGION2 0

/*
 * Start Address <0-0xFFFFFFE0>
 */
#define SAU_INIT_START2 0x00000000

/*
 * End Address <0x1F-0xFFFFFFFF>
 */
#define SAU_INIT_END2 0x00000000

/*
 * Region is
 *  0->Non-Secure
 *  1->Secure, Non-Secure Callable
 */
#define SAU_INIT_NSC2 0

/* Initialize SAU Region 3
 * Setup SAU Region 3 memory attributes
 */
#define SAU_INIT_REGION3 0

/*
 * Start Address <0-0xFFFFFFE0>
 */
#define SAU_INIT_START3 0x00000000

/*
 * End Address <0x1F-0xFFFFFFFF>
 */
#define SAU_INIT_END3 0x00000000

/*
 * Region is
 *  0->Non-Secure
 *  1->Secure, Non-Secure Callable
 */
#define SAU_INIT_NSC3 0

/*
 * Initialize SAU Region 4
 * Setup SAU Region 4 memory attributes
 */
#define SAU_INIT_REGION4 0

/*
 * Start Address <0-0xFFFFFFE0>
 */
#define SAU_INIT_START4 0x00000000 /* start address of SAU region 4 */

/*
 * End Address <0x1F-0xFFFFFFFF>
 */
#define SAU_INIT_END4 0x00000000 /* end address of SAU region 4 */

/*
 * Region is
 *  0->Non-Secure
 *  1->Secure, Non-Secure Callable
 */
#define SAU_INIT_NSC4 0

/*
 * Initialize SAU Region 5
 * Setup SAU Region 5 memory attributes
 */
#define SAU_INIT_REGION5 0

/*
 * Start Address <0-0xFFFFFFE0>
 */
#define SAU_INIT_START5 0x00000000

/*
 * End Address <0x1F-0xFFFFFFFF>
 */
#define SAU_INIT_END5 0x00000000

/*
 * Region is
 *  0->Non-Secure
 *  1->Secure, Non-Secure Callable
 */
#define SAU_INIT_NSC5 0

/*
 * Initialize SAU Region 6
 * Setup SAU Region 6 memory attributes
 */
#define SAU_INIT_REGION6 0

/*
 * Start Address <0-0xFFFFFFE0>
 */
#define SAU_INIT_START6 0x00000000

/*
 * End Address <0x1F-0xFFFFFFFF>
 */
#define SAU_INIT_END6 0x00000000

/*
 * Region is
 *  0->Non-Secure
 *  1->Secure, Non-Secure Callable
 */
#define SAU_INIT_NSC6 0

/*
 * Initialize SAU Region 7
 * Setup SAU Region 7 memory attributes
 */
#define SAU_INIT_REGION7 0

/*
 * Start Address <0-0xFFFFFFE0>
 */
#define SAU_INIT_START7 0x00000000

/*
 * End Address <0x1F-0xFFFFFFFF>
 */
#define SAU_INIT_END7 0x00000000

/*
 * Region is
 *  0->Non-Secure
 *  1->Secure, Non-Secure Callable
 */
#define SAU_INIT_NSC7 0

/*
 * max 128 SAU regions.
 * SAU regions are defined in partition.h
 */

#define SAU_INIT_REGION(n)                                                                         \
	SAU->RNR = (n & SAU_RNR_REGION_Msk);                                                       \
	SAU->RBAR = (SAU_INIT_START##n & SAU_RBAR_BADDR_Msk);                                      \
	SAU->RLAR = (SAU_INIT_END##n & SAU_RLAR_LADDR_Msk) |                                       \
		    ((SAU_INIT_NSC##n << SAU_RLAR_NSC_Pos) & SAU_RLAR_NSC_Msk) | 1U

/*
 * \brief   Setup a SAU Region
 * \details Writes the region information contained in SAU_Region to the
 *	   registers SAU_RNR, SAU_RBAR, and SAU_RLAR
 */
__STATIC_INLINE void alif_tz_sau_setup(void)
{

#if defined(__SAUREGION_PRESENT) && (__SAUREGION_PRESENT == 1U)

#if defined(SAU_INIT_REGION0) && (SAU_INIT_REGION0 == 1U)
	SAU_INIT_REGION(0);
#endif

#if defined(SAU_INIT_REGION1) && (SAU_INIT_REGION1 == 1U)
	SAU_INIT_REGION(1);
#endif

#if defined(SAU_INIT_REGION2) && (SAU_INIT_REGION2 == 1U)
	SAU_INIT_REGION(2);
#endif

#if defined(SAU_INIT_REGION3) && (SAU_INIT_REGION3 == 1U)
	SAU_INIT_REGION(3);
#endif

#if defined(SAU_INIT_REGION4) && (SAU_INIT_REGION4 == 1U)
	SAU_INIT_REGION(4);
#endif

#if defined(SAU_INIT_REGION5) && (SAU_INIT_REGION5 == 1U)
	SAU_INIT_REGION(5);
#endif

#if defined(SAU_INIT_REGION6) && (SAU_INIT_REGION6 == 1U)
	SAU_INIT_REGION(6);
#endif

#if defined(SAU_INIT_REGION7) && (SAU_INIT_REGION7 == 1U)
	SAU_INIT_REGION(7);
#endif

	/* repeat this for all possible SAU regions */
#endif /* defined (__SAUREGION_PRESENT) && (__SAUREGION_PRESENT == 1U) */

#if defined(SAU_INIT_CTRL) && (SAU_INIT_CTRL == 1U)
	SAU->CTRL = ((SAU_INIT_CTRL_ENABLE << SAU_CTRL_ENABLE_Pos) & SAU_CTRL_ENABLE_Msk) |
		    ((SAU_INIT_CTRL_ALLNS << SAU_CTRL_ALLNS_Pos) & SAU_CTRL_ALLNS_Msk);
#endif
}

#endif /* PARTITION_M55_HE_H_ */
