#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(test, LOG_LEVEL_INF);

void main(void)
{
    const struct flash_area *pfa;
    int rc = flash_area_open(FIXED_PARTITION_ID(storage_partition), &pfa);
    if (rc < 0) {
        LOG_ERR("flash_area_open failed: %d", rc);
        return;
    }

    LOG_INF("Partition @0x%x size=%u on %s",
            (unsigned)pfa->fa_off,
            (unsigned)pfa->fa_size,
            pfa->fa_dev->name);

    /* Try erase one block (0x400 bytes) */
    //rc = flash_area_erase(pfa, 0, 0x400);
    //LOG_INF("erase rc=%d", rc);

    /* Try write 16 bytes */
    uint8_t tmp[16];
    for (int i = 0; i < sizeof(tmp); i++) {
        tmp[i] = 0xA5 + i;
    }
    rc = flash_area_write(pfa, 0, tmp, sizeof(tmp));
    LOG_INF("write rc=%d", rc);

    /* Close the flash area */
    flash_area_close(pfa);
}

