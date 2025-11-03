/*
 * Copyright (c) 2021 Arm Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "audio_data.h"
#include "rnnoise_config.h"
#include <zephyr/console/console.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MainLoop, LOG_LEVEL_INF);

#include "UseCaseHandler.hpp"

int main_loop() {
    console_getline_init();
    LOG_INF("RNNoise (Zephyr) starting");

    if (!RN_UseCase_Init()) {
        LOG_ERR("RNNoise init failed");
        return -1;
    }

    // Example: process frames from a test buffer (replace with I2S ring buffer)
    extern const int16_t audio_data[];
    int16_t out_frame[FRAME_SIZE];

    for (int i = 0; i + FRAME_SIZE <= AUDIO_DATA_LENGTH; i += FRAME_SIZE) {
        float vad;
        RN_UseCase_ProcessFrame(&audio_data[i], out_frame, &vad);
        // You could stream out_frame or analyze VAD here.
    }

    LOG_INF("RNNoise done");
    return 0;
}
