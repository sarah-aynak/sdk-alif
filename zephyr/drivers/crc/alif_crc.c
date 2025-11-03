/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_alif_crc

#include <errno.h>
#include <stdint.h>
#include <zephyr/drivers/crc/alif_crc.h>
#include <zephyr/kernel.h>
#include "alif_crc_reg.h"
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

uint32_t reg_value;

#define CONF_CRC_INIT_PRIORITY	40

/**
 * @fn		crc_bit_reflect(uint32_t input)
 * @brief	Reflect the CRC 32 bit output
 * @param[in]	input	: 32 bit CRC output
 * @return	result of reflected CRC 32 bit output
 */
uint32_t crc_bit_reflect(uint32_t input)
{
	uint32_t res = 0;
	uint32_t i, bit;

	for (i = 0; i < 32; i++) {
		bit = (input >> i) & 1;
		bit = bit << (32 - (i + 1));
		res |= bit;
	}

	return res;
}

/**
 * @fn		uint32_t crc_calculate_unaligned(uint32_t key,const
 *						 uint8_t *input,
						 uint32_t length,
						 uint32_t poly)
 * @brief	To calculate the CRC result for unaligned data
			1. It will take the aligned data for CRC result from the hardware.
			2. Unaligned input data and its length.
			3. If the algorithm is 32 bit CRC then it will take the standard
				32 bit CRC Polynomial
			4. If the algorithm is 32 bit Custom CRC polynomial , it will take
				the polynomial entered from the user.
 * @param[in]  dev   : pointer to Runtime device structure
 * @param[in]  key   : Output of aligned data for CRC from the hardware
 * @param[in]  input : unaligned input data
 * @param[in]  length: length of unaligned data
 * @param[in]  poly  : Standard polynomial or the user entered polynomial
					depending upon the CRC algorithm
 * @return	Calculated CRC output for unaligned data
 */
uint32_t crc_calculate_unaligned(uint32_t key, const uint8_t *input,
				 uint32_t length, uint32_t poly)
{
	uint32_t crc, check_bit, polynomial;
	uint8_t  data;
	uint32_t i, j;

	crc = key;

	/* Store the reflected polynomial  */
	polynomial = crc_bit_reflect(poly);

	for (i = 0; i < length; i++) {
		data = input[i];
		for (j = 0; j < 8; j++) {
			check_bit = (crc ^ data) & 1;
			crc >>= 1;

			if (check_bit) {
				crc = crc ^ polynomial;
			}
			data >>= 1;
		}
	}
	return ~crc;
}

/**
 * @fn		void crc_calculate_8bit(const struct device *dev, crc_params *params)
 * @brief	Calculate the CRC output  for 8 bit CRC algorithm.
 * @param[in]   dev	: pointer to Runtime device structure
 * @param[in]   params  : pointer to crc_params structure which has length of the
			  input buffer
 * @return	 None
 */
static void crc_calculate_8bit(const struct device *dev, struct crc_params *params)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	for (int count = 0; count < params->len ; count++) {
		sys_write8(((const uint8_t *)params->data_in)[count], reg_base + CRC_DATA_IN_8_0);
	}

	/* Store the CRC output  */
	*params->data_out = sys_read32(reg_base + CRC_OUT);
}

/**
 * @fn		void crc_calculate_16bit(const struct device *dev, crc_params *params)
 * @brief	Calculate the CRC output  for 16 bit CRC algorithm.
 * @param[in]   dev	: pointer to Runtime device structure
 * @param[in]   params  : pointer to crc_params structure which has length of the
			  input buffer
 * @return	 None
 */
static void crc_calculate_16bit(const struct device *dev, struct crc_params *params)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	for (int count = 0; count < params->len ; count++) {
		sys_write8(((const uint8_t *)params->data_in)[count], reg_base + CRC_DATA_IN_8_0);
	}

	/* Store the CRC output  */
	*params->data_out =  sys_read32(reg_base + CRC_OUT);
}

/**
 * @fn		void crc_calculate_32bit(const struct device *dev, crc_params *params)
 * @brief	Calculate the CRC output  for 32 bit CRC algorithm.
 * @param[in]   dev	: pointer to Runtime device structure
 * @param[in]   params  : pointer to crc_params structure which has length of the
			  input buffer
 * @return	 None
 */
static void crc_calculate_32bit(const struct device *dev, struct crc_params *params)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	const uint32_t *data32;
	uint32_t value;
	uint32_t aligned_length = params->len - (params->len % 4);

	data32 = (const uint32_t *)params->data_in;

	for (int count = 0; count < aligned_length / 4 ; count++) {
		value = *(data32++);
		sys_write32(value, reg_base + CRC_DATA_IN_32_0);
	}

	/* Store the CRC output  */
	*params->data_out =  sys_read32(reg_base + CRC_OUT);
}

/**
 * @fn		void crc_calculate_32bit_unaligned_sw(const struct device *dev,
		crc_params *params)
 * @brief	Calculate the 32bit CRC output for the unaligned part.
 * @param[in]   dev	: pointer to Runtime device structure
 * @param[in]   params  : pointer to crc_params structure which has reflect, invert
			  and custom polynomial parameters
 * @return	 None
 */
static void crc_calculate_32bit_unaligned_sw(const struct device *dev, struct crc_params *params)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t  custom;
	uint32_t unaligned_length = (params->len % 4);
	uint32_t aligned_length = params->len - (params->len % 4);
	const uint8_t *input = (const uint8_t *)params->data_in + aligned_length;

	if (unaligned_length > 0) {
		/* Check for the custom polynomial bit   */
		if (params->custum_poly) {
			/* add the user polynomial  */
			custom = sys_read32(reg_base + CRC_POLY_CUSTOM);
		} else {
			/* add the 32 bit CRC standard polynomial  */
			custom = CRC_STANDARD_POLY;
		}

		if (params->invert) {
			*params->data_out = ~(*params->data_out);
		}

		if (!(params->reflect)) {
			*params->data_out = crc_bit_reflect(*params->data_out);
		}

		/* Calculate the CRC for unaligned data  */
		*params->data_out = crc_calculate_unaligned(*params->data_out,
							    input,
							    unaligned_length,
							    custom);

		if (!(params->reflect)) {
			*params->data_out = crc_bit_reflect(*params->data_out);
		}

		if (!(params->invert)) {
			*params->data_out = ~(*params->data_out);
		}
	}
}

/**
 * @fn		void crc_enable_8bit(const struct device *dev, crc_params *params)
 * @brief	Enable 8 bit CRC algorithm and size.
 * @param[in]   dev	: pointer to Runtime device structure
 * @return	None
 */
static inline void crc_enable_8bit(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	reg_value |= (CRC_8_CCITT | CRC_ALGO_8_BIT_SIZE);
	sys_write32(reg_value, reg_base + CRC_CONTROL);

}

/**
 * @fn		void crc_enable_16bit(const struct device *dev, crc_params *params)
 * @brief	Enable 16 bit CRC algorithm and size.
 * @param[in]   dev	: pointer to Runtime device structure
 * @return	None
 */
static inline void crc_enable_16bit(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	reg_value |= (CRC_16_CCITT | CRC_ALGO_16_BIT_SIZE);
	sys_write32(reg_value, reg_base + CRC_CONTROL);
}

/**
 * @fn		void crc_enable_32bit(const struct device *dev, crc_params *params)
 * @brief	Enable 32 bit CRC algorithm and size.
 * @param[in]   params  : if custum_poly is enabled then enable CRC32 custom polynomial
			  else enable CRC32 algorithm and size.
 * @param[in]   dev	: pointer to Runtime device structure
 * @return	None
 */
static inline void crc_enable_32bit(const struct device *dev, struct crc_params *params)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	if (params->custum_poly) {
		/* enable 32 bit poly custom CRC  */
		reg_value |= CRC_32C;
		sys_write32(reg_value, reg_base + CRC_CONTROL);
	} else {
		/* enable 32 bit CRC  */
		reg_value |= CRC_32;
		sys_write32(reg_value, reg_base + CRC_CONTROL);
	}

	/* enable 32 bit algorithm size  */
	reg_value |= CRC_ALGO_32_BIT_SIZE;
	sys_write32(reg_value, reg_base + CRC_CONTROL);
}

/**
 * @fn		void crc_params_init(const struct device *dev,uint32_t seed_value)
 * @brief	Initailizing crc parameters
 * @param[in]   dev	 : pointer to Runtime device structure
 * @param[in]   params   : pointer to crc_params which has input buffer, length of
			   the input buffer, pointer to crc output, reflect, invert,
			   bit swap, byte swap and custom polynomial parameters
 * @return	None
 */
static void crc_params_init(const struct device *dev, struct crc_params *params)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	/* Enable the CRC reflect  */
	if (params->reflect == TRUE) {
		reg_value |= CRC_REFLECT;
		sys_write32(reg_value, reg_base + CRC_CONTROL);
	}
	/* Disable the CRC reflect  */
	if (params->reflect == FALSE) {
		reg_value &= ~CRC_REFLECT;
		sys_write32(reg_value, reg_base + CRC_CONTROL);
	}
	/* Enable the CRC Invert  */
	if (params->invert == TRUE) {
		reg_value |= CRC_INVERT;
		sys_write32(reg_value, reg_base + CRC_CONTROL);
	}
	/* Disable the CRC Invert  */
	if (params->invert == FALSE) {
		reg_value &= ~CRC_INVERT;
		sys_write32(reg_value, reg_base + CRC_CONTROL);
	}
	/* Enable the CRC bit swap  */
	if (params->bit_swap == TRUE) {
		reg_value |= CRC_BIT_SWAP;
		 sys_write32(reg_value, reg_base + CRC_CONTROL);
	}
	/* Disable the CRC bit swap  */
	if (params->bit_swap == FALSE) {
		reg_value &= ~CRC_BIT_SWAP;
		sys_write32(reg_value, reg_base + CRC_CONTROL);
	}
	/* Enable the CRC byte swap  */
	if (params->byte_swap == TRUE) {
		reg_value |= CRC_BYTE_SWAP;
		sys_write32(reg_value, reg_base + CRC_CONTROL);
	}
	/* Disable the CRC byte swap  */
	if (params->byte_swap == FALSE) {
		reg_value &= ~CRC_BYTE_SWAP;
		sys_write32(reg_value, reg_base + CRC_CONTROL);
	}
	/* Enable the CRC custom polynomial  */
	if (params->custum_poly == TRUE) {
		reg_value |= CRC_CUSTOM_POLY;
		sys_write32(reg_value, reg_base + CRC_CONTROL);
	}
	/* Disable the CRC custom polynomial  */
	if (params->custum_poly == FALSE) {
		reg_value &= ~CRC_CUSTOM_POLY;
		sys_write32(reg_value, reg_base + CRC_CONTROL);
	}
}

/**
 * @fn		int crc_compute_start (const struct device *dev, crc_params *params)
 * @brief		1.calculate the CRC result for 8 bit 16 bit and 32 bit CRC algorithm.
			2.For 8 bit and 16 bit CRC algorithm our hardware can able to
			calculate the CRC result for both aligned and unaligned CRC input
			data by loading the CRC inputs in DATA_IN_8 bit register.
			3.For 32 bit CRC our hardware will support for aligned data to
			calculate the CRC Result.
			4.For unaligned data CRC_calculate_Unaligned function will calculate
			the CRC result for unaligned CRC input
			5. In CRC_calculate_Unaligned function load the aligned CRC result from
			the hardware ,unaligned CRC input,length of unaligned input data and
			the polynomial for the 32 bit CRC
 * @param[in]  dev	: pointer to Runtime device structure
 * @param[in]  params   : pointer to crc_params which has input buffer, length of the
			  input buffer, pointer to crc output, reflect, invert, bit
			  swap, byte swap and custom polynomial parameters
 * @return	 None
 */
int crc_compute_start(const struct device *dev, struct crc_params *params)
{
	struct crc_data *data = dev->data;

	/* Initailizing crc parameters  */
	crc_params_init(dev, params);

	switch (data->crc_algo) {
	/* 8 bit CRC  */
	case CRC_8_BIT_SIZE:

		/* enable 8 bit CRC algorithm and size  */
		crc_enable_8bit(dev);

		/* Calculate the CRC output  for 8 bit CRC algorithm  */
		crc_calculate_8bit(dev, params);

		break;

		/* 16 bit CRC  */
	case CRC_16_BIT_SIZE:

		/* enable 16 bit CRC algorithm and size  */
		crc_enable_16bit(dev);

		/* Calculate the CRC output  for 16 bit CRC algorithm  */
		crc_calculate_16bit(dev, params);

		break;

		/* 32 bit CRC  */
	case CRC_32_BIT_SIZE:

		/* enable 32 bit CRC algorithm and size  */
		crc_enable_32bit(dev, params);

		/* Calculate the CRC output  for 32 bit CRC algorithm  */
		crc_calculate_32bit(dev, params);

		/* Calculate the 32bit CRC output for the unaligned part  */
		crc_calculate_32bit_unaligned_sw(dev, params);

		break;
	}
	return 0;
}

/**
 * @fn		void crc_set_seed_val(const struct device *dev,uint32_t seed_value)
 * @brief	Set crc seed value
 * @param[in]   seed_value : Seed value depending on whether the data is 8 bit
			    or 16 or 32 bit
 * @param[in]   dev	: pointer to Runtime device structure
 * @return	None
 */
int crc_set_seed_val(const struct device *dev, uint32_t seed_value)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_write32(seed_value, reg_base + CRC_SEED);

	return 0;
}

/**
 * @fn		void crc_set_custom_poly(const struct device *dev,uint32_t polynomial)
 * @brief	add polynomial value
 * @param[in]   polynomial : Polynomial data for 8 bit or 16 or 32 bit
 * @param[in]   dev	   : pointer to Runtime device structure
 * @return	None
 */
int crc_set_custom_poly(const struct device *dev, uint32_t polynomial)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_write32(polynomial, reg_base + CRC_POLY_CUSTOM);

	return 0;
}

static const struct crc_driver_api crc_api_funcs = {
		.start = crc_compute_start,
		.seed = crc_set_seed_val,
		.polynomial = crc_set_custom_poly,
};

/* Init function  */
static int crc_initialize(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	reg_value |= CRC_INIT_BIT;
	sys_write32(reg_value, reg_base + CRC_CONTROL);

	return 0;
}

/********************** Device Definition per instance Macros. ***********************/

#define CRC_INIT(n)						\
	static struct crc_data data##n = {					\
			.crc_algo = DT_INST_ENUM_IDX(n, crc_algo)	\
	};			\
	static const struct crc_config config_##n = {		\
			.base_addr = DT_INST_REG_ADDR(n),			\
	 };			\
	DEVICE_DT_INST_DEFINE(n, crc_initialize, NULL, &data##n, \
						  &config_##n, POST_KERNEL,			\
						  CONF_CRC_INIT_PRIORITY, &crc_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(CRC_INIT)
