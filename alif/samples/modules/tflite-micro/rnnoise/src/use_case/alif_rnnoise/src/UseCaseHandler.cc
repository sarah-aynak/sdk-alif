/*
 * Copyright (c) 2021-2022 Arm Limited. All rights reserved.
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
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(UseCaseRN, LOG_LEVEL_INF);

#include "RNNoiseModel.hpp"
#include "RNNoiseProcessing.hpp"
#include "rnnoise_model.h"
#include "rnnoise_config.h"

using namespace arm::app;

namespace {
constexpr size_t kArenaSize = 64 * 1024;
static uint8_t tensorArena[kArenaSize];
static RNNoiseModel g_model;

// Buffers for output
static int16_t g_outFrame[FRAME_SIZE];
static float   g_vadProb = 0.0f;
}


bool RN_UseCase_Init() {
    if (!g_model.Init(tensorArena, kArenaSize,
                      rnnoise_model, rnnoise_model_len)) {
        LOG_ERR("RN: model init failed");
        return false;
    }
    LOG_INF("RN: model initialized with %zu inputs, %zu outputs",
            g_model.GetNumInputs(), g_model.GetNumOutputs());
    for (size_t i = 0; i < g_model.GetNumInputs(); ++i) {
        auto* t = g_model.GetInputTensor(i);
        LOG_INF("Input %zu: %s", i, t->name ? t->name : "(null)");
    }
    for (size_t i = 0; i < g_model.GetNumOutputs(); ++i) {
        auto* t = g_model.GetOutputTensor(i);
        LOG_INF("Output %zu: %s", i, t->name ? t->name : "(null)");
    }
    return true;
}

bool RN_UseCase_ProcessFrame(const int16_t* pcm_in,
                             int16_t* pcm_out,
                             float* vad_out) {
    // Pre-process: PCM -> features -> input tensor
    RNNoisePreProcess pre(g_model.GetInputTensor(0));
    if (!pre.DoPreProcess(pcm_in, FRAME_SIZE * sizeof(int16_t))) {
        LOG_ERR("RN: PreProcess failed");
        return false;
    }

    // Run inference
    if (!g_model.RunInference()) {
        LOG_ERR("RN: Inference failed");
        return false;
    }

    // --- NEW: GRU state rollover ---
    {
        auto* out = g_model.GetOutputTensor(0);  // Identity_int8 (96)
        auto* in  = g_model.GetInputTensor(1);
        std::memcpy(in->data.int8, out->data.int8, in->bytes);
    }
    {
        auto* out = g_model.GetOutputTensor(2);  // Identity_2_int8 (48)
        auto* in  = g_model.GetInputTensor(2);
        std::memcpy(in->data.int8, out->data.int8, in->bytes);
    }
    {
        auto* out = g_model.GetOutputTensor(3);  // Identity_3_int8 (24)
        auto* in  = g_model.GetInputTensor(3);
        std::memcpy(in->data.int8, out->data.int8, in->bytes);
    }

    // Post-process: outputs -> denoised PCM + VAD
    RNNoisePostProcess post(g_model.GetOutputTensor(1),   // gains
                            g_model.GetOutputTensor(4),   // vad prob
                            g_outFrame);
    if (!post.DoPostProcess()) {
        LOG_ERR("RN: PostProcess failed");
        return false;
    }

    // Copy results out
    for (int i = 0; i < FRAME_SIZE; ++i) {
        pcm_out[i] = g_outFrame[i];
    }
    if (vad_out) {
        *vad_out = post.GetVadProb();
    }

    return true;
}

 // extern "C"
