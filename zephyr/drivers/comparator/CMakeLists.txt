# Copyright (C) 2024 Alif Semiconductor.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_CMP_ALIF  alif_cmp.c)

