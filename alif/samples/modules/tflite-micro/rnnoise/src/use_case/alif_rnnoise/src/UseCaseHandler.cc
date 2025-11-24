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

constexpr size_t kArenaSize = 64 * 1024;
static uint8_t tensorArena[kArenaSize];
static RNNoiseModel g_model;

// Buffers for output
static int16_t g_outFrame[FRAME_SIZE];
static float   g_vadProb = 0.0f;


bool RN_UseCase_Init() {
    if (!g_model.Init(tensorArena, kArenaSize,
                      rnnoise_model, rnnoise_model_len)) {
        //LOG_ERR("RN: model init failed");
        return false;
    }
    for (size_t i = 0; i < g_model.GetNumInputs(); ++i) {
        auto* t = g_model.GetInputTensor(i);
        //LOG_INF("Input %zu: %s (%d bytes)", i, t && t->name ? t->name : "(null)", t ? t->bytes : -1);
    }
    for (size_t i = 0; i < g_model.GetNumOutputs(); ++i) {
        auto* t = g_model.GetOutputTensor(i);
        //LOG_INF("Output %zu: %s (%d bytes)", i, t && t->name ? t->name : "(null)", t ? t->bytes : -1);
    }
    //LOG_INF("RN: model initialized with %zu inputs, %zu outputs",
    //        g_model.GetNumInputs(), g_model.GetNumOutputs());
    for (size_t i = 0; i < g_model.GetNumInputs(); ++i) {
        auto* t = g_model.GetInputTensor(i);
        //LOG_INF("Input %zu: %s", i, t->name ? t->name : "(null)");
    }
    for (size_t i = 0; i < g_model.GetNumOutputs(); ++i) {
        auto* t = g_model.GetOutputTensor(i);
        //LOG_INF("Output %zu: %s", i, t->name ? t->name : "(null)");
    }
    return true;
}

bool RN_UseCase_ProcessFrame(const int16_t* pcm_in,
                             int16_t* pcm_out,
                             float* vad_out) {
    // Pre-process: PCM -> features -> input tensor
    //LOG_INF("RN: PreProcess starting");
    static RNNoisePreProcess pre;
    if (!pre.DoPreProcess(g_model.GetInputTensor(0), pcm_in, FRAME_SIZE * sizeof(int16_t))) {
        //LOG_ERR("RN: PreProcess failed");
        return false;
    }
    //LOG_INF("RN: PreProcess done");
    // Initialize other input tensors (1–3) before inference
    std::memset(g_model.GetInputTensor(1)->data.int8, 0, g_model.GetInputTensor(1)->bytes);  // Band gains
    std::memset(g_model.GetInputTensor(2)->data.int8, 0, g_model.GetInputTensor(2)->bytes);  // GRU state
    std::memset(g_model.GetInputTensor(3)->data.int8, 0, g_model.GetInputTensor(3)->bytes);
    // Ensure tensors 1–3 are zeroed and metadata is logged
    for (int i = 1; i <= 3; ++i) {
    	auto* t = g_model.GetInputTensor(i);
    	if (t && t->data.int8 && t->bytes > 0) {
            std::memset(t->data.int8, 0, t->bytes);
            //LOG_INF("Zeroed Input[%d] at %p (%d bytes)", i, t->data.int8, t->bytes);
            //LOG_INF("Input[%d]: type=%d, scale=%f, zp=%d, dims=[%d,%d]",
                    //i, t->type, t->params.scale, t->params.zero_point,
                    //t->dims->data[0], t->dims->data[1]);
        } else {
            //LOG_ERR("Input[%d] missing or invalid", i);
        }
    }
    // Run inference
    if (!g_model.RunInference()) {
        //LOG_ERR("RN: Inference failed");
        return false;
    }
    //LOG_INF("RN: Inference done");
    //LOG_INF("RN: GRU rollover starting");
    // GRU A/B/C blocks

    // --- NEW: GRU state rollover ---
    {
        auto* out = g_model.GetOutputTensor(0);  // Identity_int8 (96)
        auto* in  = g_model.GetInputTensor(3);
        if (in && out && in->data.int8 && out->data.int8 && in->bytes <= out->bytes) {
            std::memcpy(in->data.int8, out->data.int8, in->bytes);
        } else {
            //LOG_ERR("GRU A rollover failed");
            return false;
        }
    }
    {
        auto* out = g_model.GetOutputTensor(2);  // Identity_2_int8 (48)
        auto* in  = g_model.GetInputTensor(2);
        if (in && out && in->data.int8 && out->data.int8 && in->bytes <= out->bytes) {
            std::memcpy(in->data.int8, out->data.int8, in->bytes);
        } else {
            //LOG_ERR("GRU B rollover failed");
            return false;
        }
    }
    {
        auto* out = g_model.GetOutputTensor(3);  // Identity_3_int8 (24)
        auto* in  = g_model.GetInputTensor(1);
        if (in && out && in->data.int8 && out->data.int8 && in->bytes <= out->bytes) {
            std::memcpy(in->data.int8, out->data.int8, in->bytes);
        } else {
            //LOG_ERR("GRU C rollover failed");
            return false;
        }
    }

    // Post-process: outputs -> denoised PCM + VAD
    static RNNoisePostProcess post(g_model.GetOutputTensor(1),   // gains
                            g_model.GetOutputTensor(4),   // vad prob
                            g_outFrame);
    if (!post.DoPostProcess()) {
        //LOG_ERR("RN: PostProcess failed");
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
