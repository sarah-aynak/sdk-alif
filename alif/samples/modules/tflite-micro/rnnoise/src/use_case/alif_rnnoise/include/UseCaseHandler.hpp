/* Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

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
#ifndef ALIF_RNNOISE_EVT_HANDLER_HPP
#define ALIF_RNNOISE_EVT_HANDLER_HPP

#include <cstdint>
#include "RNNoiseModel.hpp"  // ✅ Add this
/**
 * @brief   Initialise the RNNoise model and runtime.
 * @return  true if successful, false otherwise.
 */
bool RN_UseCase_Init();

/**
 * @brief   Process one frame of audio through RNNoise.
 * @param[in]   pcm_in   Pointer to input PCM frame (FRAME_SIZE samples).
 * @param[out]  pcm_out  Pointer to output denoised PCM frame.
 * @param[out]  vad_out  Optional pointer to float where VAD probability is written.
 * @return      true if successful, false otherwise.
 */
bool RN_UseCase_ProcessFrame(const int16_t* pcm_in,
                             int16_t* pcm_out,
                             float* vad_out);

extern arm::app::RNNoiseModel g_model;
#endif /* ALIF_RNNOISE_EVT_HANDLER_HPP */
