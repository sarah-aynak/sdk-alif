/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT alif_mram_flash_controller

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <mram_rw.h>
#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_mram_alif);

#define SOC_NV_FLASH_NODE      DT_NODELABEL(mram_storage)
#define FLASH_MRAM_BASE_OFFSET DT_REG_ADDR(SOC_NV_FLASH_NODE)
#define FLASH_MRAM_ERASE_UNIT  DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)
#define FLASH_MRAM_PROG_UNIT   DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define FLASH_MRAM_FLASH_SIZE  DT_REG_SIZE(SOC_NV_FLASH_NODE)
#define FLASH_MRAM_ERASE_VALUE 0x0
#define FLASH_MRAM_PAGE_COUNT  (FLASH_MRAM_FLASH_SIZE / FLASH_MRAM_ERASE_UNIT)

#if (FLASH_MRAM_ERASE_UNIT % FLASH_MRAM_PROG_UNIT)
#error "Erase unit must be a multiple of program unit"
#endif

#define MRAM_FLASH(offset) ((uint8_t *)FLASH_MRAM_BASE_OFFSET + (offset))

struct mram_flash_data {
	/* mram flash write protection is managed in software. */
	struct k_sem lock;
};

static const struct flash_parameters flash_mram_parameters = {
	.write_block_size = FLASH_MRAM_PROG_UNIT,
	.erase_value = FLASH_MRAM_ERASE_VALUE
};

/**
 * @brief check if offset and len are in valid range.
 *
 * @param dev Pointer to device driver instance
 * @param offset Flash offset address for read/write operation.
 * @param len Number of bytes to read/write from offset address.
 *
 * @return 1 if range is valid, 0 otherwise.
 */
static int flash_range_is_valid(const struct device *dev, off_t offset,
				size_t len)
{
	ARG_UNUSED(dev);
	if (((offset + len) > FLASH_MRAM_FLASH_SIZE) || (offset < 0x0))
		return 0;
	return 1;
}

/**
 * @brief read data from the flash from the given address.
 *
 * @param dev Pointer to device driver instance
 * @param offset Flash offset address at which read starts.
 * @param data Pointer to the uint8_t buffer to store read data.
 * @param len Number of bytes to read from offset address.
 *
 * @return 0 if successful.
 * @return -EINVAL if offset or len is out of range.
 * @return -EACCES if unable to take semaphore.
 */
static int flash_mram_read(const struct device *dev, const off_t offset,
			   void *data, const size_t len)
{
	struct mram_flash_data *dev_data = dev->data;

	if (!flash_range_is_valid(dev, offset, len)) {
		LOG_ERR("mram_read: Invalid range offset: %ld len: %d\n",
			(long)offset, len);
		return -EINVAL;
	}
	if (k_sem_take(&dev_data->lock, K_FOREVER)) {
		LOG_ERR("mram_read: unable to take semaphore\n");
		return -EACCES;
	}
	/* Reading from MRAM does not have 16 bytes alignment or */
	/* the multiple of 16 read size constraints */
	memcpy(data, MRAM_FLASH(offset), len);
	k_sem_give(&dev_data->lock);

	return 0;
}

/**
 * @brief write data of FLASH_MRAM_PROG_UNIT bytes into MRAM flash.
 *
 * @param unit Address to write FLASH_MRAM_PROG_UNIT bytes of data.
 * @param data Pointer to the uint8_t buffer containing data for write.
 *
 */
static void mram_unit_write(const uint32_t unit, uint8_t *data)
{
	const off_t unit_addr = (unit);

	/* Write 16 bytes of data in MRAM */
	write_16bytes(MRAM_FLASH(unit_addr), data);
}

/**
 * @brief write data into the flash from the given address.
 *
 * @param dev Pointer to device driver instance.
 * @param offset Flash offset address at which write starts.
 * @param data Pointer to the uint8_t buffer containing data for write.
 * @param len Number of bytes to write from offset address.
 *
 * @return 0 if successful.
 * @return -EINVAL if offset or len is out of range.
 * @return -EACCES if unable to take semaphore.
 */
static int flash_mram_write(const struct device *dev, const off_t offset,
			     const void *data, const size_t len)
{
	struct mram_flash_data *dev_data = dev->data;
	uint32_t i;
	uint32_t unit_start;

	if (!flash_range_is_valid(dev, offset, len)) {
		LOG_ERR("mram_write: Invalid range offset: %ld len: %d\n",
			(long)offset, len);
		return -EINVAL;
	}

	if ((offset % FLASH_MRAM_PROG_UNIT) ||
	   (len % FLASH_MRAM_PROG_UNIT)) {
		LOG_ERR("mram_write: offset %ld and len %d "
			"must be multiple of %d\n",
			(long)offset, len, FLASH_MRAM_PROG_UNIT);
		return -EINVAL;
	}
	unit_start = offset;
	if (k_sem_take(&dev_data->lock, K_FOREVER)) {
		LOG_ERR("mram_write: unable to take semaphore\n");
		return -EACCES;
	}
	for (i = 0 ; i < len ; i = i + FLASH_MRAM_PROG_UNIT) {
		mram_unit_write((unit_start + i),
				((uint8_t *)data + (i)));
	}
	k_sem_give(&dev_data->lock);

	return 0;
}

/**
 * @brief erase data of FLASH_MRAM_ERASE_UNIT bytes from the MRAM flash.
 *
 * @param unit Address to erase FLASH_MRAM_ERASE_UNIT bytes.
 *             The erase value is defined by FLASH_MRAM_ERASE_VALUE.
 */
static void mram_unit_erase(const uint32_t unit)
{
	const off_t unit_addr = (unit * FLASH_MRAM_ERASE_UNIT);
	uint32_t i;

	for (i = unit_addr; i < (unit_addr + FLASH_MRAM_ERASE_UNIT);
	    i += FLASH_MRAM_PROG_UNIT) {
		/* Erase 16 bytes of MRAM data */
		erase_16bytes(MRAM_FLASH(i));
	}
}

/**
 * @brief erase data from the flash at the given address.
 *
 * @param dev Pointer to device driver instance.
 * @param offset Flash offset address at which erase starts.
 * @param len Number of bytes to erase from offset address.
 *
 * @return 0 if successful.
 * @return -EINVAL if offset or len is out of range.
 * @return -EACCES if unable to take semaphore.
 */
static int flash_mram_erase(const struct device *dev, off_t offset,
			    size_t len)
{
	struct mram_flash_data *dev_data = dev->data;
	uint32_t mram_unit_start;
	uint32_t i;

	if (!flash_range_is_valid(dev, offset, len)) {
		LOG_ERR("mram_erase: Invalid range offset: %ld len :%d\n",
			(long)offset, len);
		return -EINVAL;
	}
	if ((offset % FLASH_MRAM_ERASE_UNIT) ||
	   (len % FLASH_MRAM_ERASE_UNIT)) {
		LOG_ERR("mram_erase: offset %ld and len %d "
			"must be multiple of %d\n",
			(long)offset, len, FLASH_MRAM_ERASE_UNIT);
		return -EINVAL;
	}
	mram_unit_start = offset / FLASH_MRAM_ERASE_UNIT;
	if (k_sem_take(&dev_data->lock, K_FOREVER)) {
		LOG_ERR("mram_erase: unable to take semaphore\n");
		return -EACCES;
	}
	for (i = 0 ; i < len / FLASH_MRAM_ERASE_UNIT ; ++i) {
		mram_unit_erase(mram_unit_start + i);
	}
	k_sem_give(&dev_data->lock);

	return 0;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_mram_pages_layout = {
	.pages_count = FLASH_MRAM_PAGE_COUNT,
	.pages_size = FLASH_MRAM_ERASE_UNIT,
};

static void flash_mram_page_layout(const struct device *dev,
				  const struct flash_pages_layout **layout,
				  size_t *layout_size)
{
	*layout = &flash_mram_pages_layout;
	*layout_size = 1;
}
#endif

static const struct flash_parameters *
flash_mram_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_mram_parameters;
}

static const struct flash_driver_api flash_mram_api = {
	.read = flash_mram_read,
	.write = flash_mram_write,
	.erase = flash_mram_erase,
	.get_parameters = flash_mram_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_mram_page_layout,
#endif
};

static int flash_mram_init(const struct device *dev)
{
	struct mram_flash_data *dev_data = dev->data;

	k_sem_init(&dev_data->lock, 1, 1);
	/* NOTE: Uncomment below line to erase storage parition of MRAM */
	/* flash_mram_erase(dev, 0x0, FLASH_MRAM_FLASH_SIZE); */
	return 0;
}
static struct mram_flash_data data;

DEVICE_DT_INST_DEFINE(0, flash_mram_init, NULL,
			&data, NULL,
			POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
			&flash_mram_api);
