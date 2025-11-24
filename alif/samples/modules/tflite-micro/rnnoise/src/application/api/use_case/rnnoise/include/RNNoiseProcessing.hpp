/*
 * SPDX-FileCopyrightText: Copyright 2025
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "BaseProcessing.hpp"
#include "Model.hpp"
#include "rnnoise_config.h"
#include <vector>
#include <string>

namespace arm {
namespace app {

/**
 * @brief RNNoise pre-processing: converts PCM frames into RNNoise features
 *        and writes them into the model input tensor.
 */
class RNNoisePreProcess {
public:
    RNNoisePreProcess() = default;

    bool DoPreProcess(TfLiteTensor* inputTensor,
                      const void* data,
                      size_t inputSize);
};

/**
 * @brief RNNoise post-processing: reads model outputs (gains, VAD),
 *        applies DSP synthesis, and writes denoised PCM.
 */
class RNNoisePostProcess : public BasePostProcess {
public:
    RNNoisePostProcess(TfLiteTensor* gainsTensor,
                       TfLiteTensor* vadTensor,
                       int16_t* pcmOut);
    bool DoPostProcess() override;

    float GetVadProb() const;

private:
    TfLiteTensor* m_gainsTensor;
    TfLiteTensor* m_vadTensor;
    int16_t* m_pcmOut;
    float m_vadProb{0.0f};
};

} // namespace app
} // namespace arm
