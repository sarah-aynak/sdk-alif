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
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
LOG_MODULE_REGISTER(MainLoop, LOG_LEVEL_INF);

#include "UseCaseHandler.hpp"

#define SAMPLE_RATE    48000
#define FRAME_SIZE     480
#define SECONDS 1
#define NUM_FRAMES     (AUDIO_DATA_LENGTH / FRAME_SIZE)
#define TOTAL_SAMPLES  (SAMPLE_RATE * SECONDS)

static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart4));
static int16_t pcm_buffer[TOTAL_SAMPLES];   // ~64 KB buffer

// Example: send WAV header
void send_wav_header(const struct device *uart, int sample_rate,
                     int channels, int bits_per_sample, int num_samples) {
    uint32_t data_size = num_samples * channels * (bits_per_sample / 8);
    uint32_t byte_rate = sample_rate * channels * (bits_per_sample / 8);
    uint16_t block_align = channels * (bits_per_sample / 8);

    const uint8_t header[44] = {
        'R','I','F','F',
        (uint8_t)(data_size + 36), (uint8_t)((data_size + 36) >> 8),
        (uint8_t)((data_size + 36) >> 16), (uint8_t)((data_size + 36) >> 24),
        'W','A','V','E','f','m','t',' ',
        16,0,0,0,   // PCM chunk size
        1,0,        // format = PCM
        (uint8_t)channels, (uint8_t)(channels >> 8),
        (uint8_t)sample_rate, (uint8_t)(sample_rate >> 8),
        (uint8_t)(sample_rate >> 16), (uint8_t)(sample_rate >> 24),
        (uint8_t)byte_rate, (uint8_t)(byte_rate >> 8),
        (uint8_t)(byte_rate >> 16), (uint8_t)(byte_rate >> 24),
        (uint8_t)block_align, (uint8_t)(block_align >> 8),
        (uint8_t)bits_per_sample, (uint8_t)(bits_per_sample >> 8),
        'd','a','t','a',
        (uint8_t)data_size, (uint8_t)(data_size >> 8),
        (uint8_t)(data_size >> 16), (uint8_t)(data_size >> 24)
    };

    for (int i = 0; i < sizeof(header); i++) {
        uart_poll_out(uart, header[i]);
    }
}
int main_loop() {
    //console_getline_init();
    //LOG_INF("RNNoise (Zephyr) starting");
    if (!RN_UseCase_Init()) {
        LOG_ERR("RNNoise init failed");
        return -1;
    }
    size_t unused;
    //if (k_thread_stack_space_get(k_current_get(), &unused) == 0) {
    //    LOG_INF("Unused stack space inside main_loop: %zu bytes", unused);
    //}
    // Example: process frames from a test buffer (replace with I2S ring buffer)
    extern const int16_t audio_data[];
    int16_t out_frame[FRAME_SIZE];

    for (int i = 0; i + FRAME_SIZE <= AUDIO_DATA_LENGTH; i += FRAME_SIZE) {
        float vad;
        RN_UseCase_ProcessFrame(&audio_data[i], out_frame, &vad);
        // You could stream out_frame or analyze VAD here.
        // Append this frame into the big buffer
        memcpy(&pcm_buffer[(i / FRAME_SIZE) * FRAME_SIZE],
               out_frame,
               FRAME_SIZE * sizeof(int16_t));
        size_t unused;
        //if (k_thread_stack_space_get(k_current_get(), &unused) == 0) {
        //    LOG_INF("Unused stack after frame %d: %zu bytes", i / FRAME_SIZE, unused);
        //}
    }
    //if (!device_is_ready(uart_dev)) {
    //    printk("UART device not ready!\n");
    //    return -1;
    //}
    //uart_poll_out(uart_dev, 'H');
    //uart_poll_out(uart_dev, 'I');
    //uart_poll_out(uart_dev, '\n');
    // Send WAV header + full buffer once
    send_wav_header(uart_dev, SAMPLE_RATE, 1, 16, TOTAL_SAMPLES);
    //printk("Sending %d samples\n", TOTAL_SAMPLES);
    for (int j = 0; j < TOTAL_SAMPLES; j++) {
        int16_t sample = pcm_buffer[j];
        uart_poll_out(uart_dev, sample & 0xFF);        // LSB
        uart_poll_out(uart_dev, (sample >> 8) & 0xFF); // MSB
    }
    //LOG_INF("RNNoise done");
    return 0;
}
