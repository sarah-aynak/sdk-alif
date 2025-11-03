/*
 * SPDX-License-Identifier: Apache-2.0
 */
#define _USE_MATH_DEFINES
#include "RNNoiseProcessing.hpp"
#include "rnnoise_config.h"
#include <zephyr/logging/log.h>
#include <cmath>
#include <cstring>

extern "C" {
#include "kiss_fft.h"
}

constexpr float M_PI = 3.14159265358979323846f;

LOG_MODULE_REGISTER(RNNoiseProc, LOG_LEVEL_INF);

using namespace arm::app;

/* ------------ Global processing state ------------ */
namespace {

static bool          g_init    = false;
static kiss_fft_cfg  g_fftCfg  = nullptr;
static kiss_fft_cfg  g_ifftCfg = nullptr;

static float         g_hann[FRAME_SIZE];
static int           g_barkBandOfBin[FREQ_SIZE];

// Persisted complex spectrum (positive bins) from PreProcess
static kiss_fft_cpx  g_lastSpectrum[FREQ_SIZE];
static bool          g_haveSpectrum = false;

// Simple overlap-add buffer (can be enhanced to true 50% overlap)
static float         g_overlap[FRAME_SIZE];

// Constants
constexpr float kEps   = 1e-12f;
constexpr float kFs    = static_cast<float>(SAMPLE_RATE);

/* ------------ Quantization helpers (from your model metadata) ------------ */

// Input features (42) — scale=0.221500605, zp=14
inline int8_t QuantizeInput(float x) {
    const float scale = 0.221500605f;
    const int   zp    = 14;
    int32_t q = lroundf(x / scale) + zp;
    if (q < -128) q = -128;
    if (q > 127)  q = 127;
    return static_cast<int8_t>(q);
}

// Band gains (22) and VAD (1) — scale=1/256, zp=-128
inline float DequantizeGain(int8_t q) {
    return 0.00390625f * (static_cast<float>(q) + 128.0f);
}
inline float DequantizeVad(int8_t q) {
    return 0.00390625f * (static_cast<float>(q) + 128.0f);
}

/* ------------ Init and feature extraction ------------ */

inline float HzToBark(float f) {
    float z = 26.81f * f / (1960.0f + f) - 0.53f;
    if (z < 0.0f) z = 0.0f;
    if (z > 24.0f) z = 24.0f;
    return z;
}

void InitOnce() {
    if (g_init) return;
    g_init = true;

    // Hann window
    for (int n = 0; n < FRAME_SIZE; ++n) {
        g_hann[n] = 0.5f * (1.0f - std::cos(2.0f * M_PI * n / (FRAME_SIZE - 1)));
    }
    size_t lenmem =0; 
    // FFT plans
    g_fftCfg  = kiss_fft_alloc(FRAME_SIZE, /*inverse=*/0, nullptr, (size_t*)&lenmem);
    g_ifftCfg = kiss_fft_alloc(FRAME_SIZE, /*inverse=*/1, nullptr, (size_t*)&lenmem);

    // Bark mapping: 22 bands across Bark [0..24]
    for (int k = 0; k < FREQ_SIZE; ++k) {
        float f = (kFs * k) / FRAME_SIZE;
        float z = HzToBark(f);
        int band = static_cast<int>(std::floor((z / 24.0f) * 22.0f));
        if (band < 0) band = 0;
        if (band > 21) band = 21;
        g_barkBandOfBin[k] = band;
    }

    std::memset(g_overlap, 0, sizeof(g_overlap));
    std::memset(g_lastSpectrum, 0, sizeof(g_lastSpectrum));
    g_haveSpectrum = false;
}

void ComputeSpectrum(const int16_t* pcm,
                     kiss_fft_cpx* freq,
                     float* mag) {
    static kiss_fft_cpx timeBuf[FRAME_SIZE];

    for (int n = 0; n < FRAME_SIZE; ++n) {
        float x = static_cast<float>(pcm[n]) / 32768.0f;
        x *= g_hann[n];
        timeBuf[n].r = x;
        timeBuf[n].i = 0.0f;
    }

    kiss_fft(g_fftCfg, timeBuf, freq);

    for (int k = 0; k < FREQ_SIZE; ++k) {
        float re = freq[k].r;
        float im = freq[k].i;
        mag[k] = std::sqrt(re * re + im * im) + kEps;
        g_lastSpectrum[k] = freq[k]; // persist for synthesis
    }
    g_haveSpectrum = true;
}

void ComputeBarkEnergies(const float* mag, float* barkE /* size 22 */) {
    std::memset(barkE, 0, sizeof(float) * 22);
    for (int k = 0; k < FREQ_SIZE; ++k) {
        barkE[g_barkBandOfBin[k]] += mag[k] * mag[k];
    }
    for (int b = 0; b < 22; ++b) {
        barkE[b] = std::log10(barkE[b] + kEps);
    }
}

void ComputeDCT22(const float* in22, float* out, int nOut) {
    for (int k = 0; k < nOut; ++k) {
        float sum = 0.0f;
        for (int n = 0; n < 22; ++n) {
            sum += in22[n] * std::cos(M_PI * (n + 0.5f) * k / 22.0f);
        }
        out[k] = sum;
    }
}

void ComputePitchFeatures(const int16_t* pcm,
                          float* pitchCorr,
                          float* pitchPeriodMs) {
    int minLag = static_cast<int>(kFs * 0.002f); // ~96 at 48 kHz
    int maxLag = static_cast<int>(kFs * 0.020f); // ~960
    if (maxLag >= FRAME_SIZE - 1) maxLag = FRAME_SIZE - 2;
    if (minLag < 2) minLag = 2;

    static float x[FRAME_SIZE];
    for (int n = 0; n < FRAME_SIZE; ++n) {
        x[n] = static_cast<float>(pcm[n]) / 32768.0f;
    }

    float E0 = kEps;
    for (int n = 0; n < FRAME_SIZE; ++n) E0 += x[n] * x[n];

    int bestLag = minLag;
    float bestCorr = 0.0f;

    for (int lag = minLag; lag <= maxLag; ++lag) {
        float R = 0.0f, E1 = kEps;
        int N = FRAME_SIZE - lag;
        for (int n = 0; n < N; ++n) {
            R  += x[n] * x[n + lag];
            E1 += x[n + lag] * x[n + lag];
        }
        float c = R / std::sqrt(E0 * E1);
        if (c > bestCorr) {
            bestCorr = c;
            bestLag  = lag;
        }
    }

    *pitchCorr    = bestCorr;
    *pitchPeriodMs = static_cast<float>(bestLag) * 1000.0f / kFs;
}

/* ------------ Synthesis ------------ */

void SynthesizeFrame(const float* binGains, int16_t* pcmOut) {
    static kiss_fft_cpx freqBuf[FRAME_SIZE];
    static kiss_fft_cpx timeBuf[FRAME_SIZE];

    if (!g_haveSpectrum) {
        std::memset(pcmOut, 0, sizeof(int16_t) * FRAME_SIZE);
        return;
    }

    // Apply gains to positive bins
    for (int k = 0; k < FREQ_SIZE; ++k) {
        freqBuf[k].r = g_lastSpectrum[k].r * binGains[k];
        freqBuf[k].i = g_lastSpectrum[k].i * binGains[k];
    }
    // Mirror to build full spectrum for real iFFT
    for (int k = FREQ_SIZE; k < FRAME_SIZE; ++k) {
        int km = FRAME_SIZE - k;
        freqBuf[k].r =  freqBuf[km].r;
        freqBuf[k].i = -freqBuf[km].i;
    }

    kiss_fft(g_ifftCfg, freqBuf, timeBuf);

    for (int n = 0; n < FRAME_SIZE; ++n) {
        float x = timeBuf[n].r / FRAME_SIZE;
        float y = x * g_hann[n] + g_overlap[n];

        float val = y * 32768.0f;
        if (val >  32767.f) val =  32767.f;
        if (val < -32768.f) val = -32768.f;
        pcmOut[n] = static_cast<int16_t>(val);
    }

    // Simple OLA reset — upgrade to true overlap if you use hop<frame
    std::memset(g_overlap, 0, sizeof(g_overlap));
    g_haveSpectrum = false;
}

} // namespace

/* ------------ RNNoisePreProcess ------------ */

RNNoisePreProcess::RNNoisePreProcess(TfLiteTensor* inputTensor)
    : m_inputTensor(inputTensor) {}

bool RNNoisePreProcess::DoPreProcess(const void* data, size_t inputSize) {
    InitOnce();

    if (!data || inputSize != FRAME_SIZE * sizeof(int16_t)) {
        LOG_ERR("RNNoisePreProcess: invalid input size (%zu)", inputSize);
        return false;
    }

    const int16_t* pcm_in = static_cast<const int16_t*>(data);

    // Spectrum
    kiss_fft_cpx freq[FREQ_SIZE];
    float mag[FREQ_SIZE];
    ComputeSpectrum(pcm_in, freq, mag);

    // Bark energies
    float barkE[22];
    ComputeBarkEnergies(mag, barkE);

    // Cepstra
    float cepstra[18];
    ComputeDCT22(barkE, cepstra, 18);

    // Pitch features
    float pitchCorr = 0.0f, pitchPeriodMs = 0.0f;
    ComputePitchFeatures(pcm_in, &pitchCorr, &pitchPeriodMs);

    // Assemble 42 features: 22 + 18 + 2
    float features[NB_FEATURES];
    int idx = 0;
    for (int b = 0; b < 22; ++b) features[idx++] = barkE[b];
    for (int c = 0; c < 18; ++c) features[idx++] = cepstra[c];
    features[idx++] = pitchCorr;
    features[idx++] = pitchPeriodMs;

    // Quantize to int8
    for (int i = 0; i < NB_FEATURES; ++i) {
        m_inputTensor->data.int8[i] = QuantizeInput(features[i]);
    }

    return true;
}

/* ------------ RNNoisePostProcess ------------ */

RNNoisePostProcess::RNNoisePostProcess(TfLiteTensor* gainsTensor,
                                       TfLiteTensor* vadTensor,
                                       int16_t* pcmOut)
    : m_gainsTensor(gainsTensor),
      m_vadTensor(vadTensor),
      m_pcmOut(pcmOut) {}

bool RNNoisePostProcess::DoPostProcess() {
    InitOnce();

    if (!m_gainsTensor || !m_vadTensor || !m_pcmOut) {
        LOG_ERR("RNNoisePostProcess: missing tensors/buffer");
        return false;
    }

    // Dequantize 22 band gains
    float bandGains[NB_BANDS];
    for (int i = 0; i < NB_BANDS; ++i) {
        float g = DequantizeGain(m_gainsTensor->data.int8[i]);
        // Clamp defensively
        if (g < 0.0f) g = 0.0f;
        if (g > 1.5f) g = 1.5f;
        bandGains[i] = g;
    }

    // Dequantize VAD probability
    m_vadProb = DequantizeVad(m_vadTensor->data.int8[0]);
    LOG_INF("RNNoise VAD=%.3f", m_vadProb);

    // Per-bin gains via Bark mapping
    float binGains[FREQ_SIZE];
    for (int k = 0; k < FREQ_SIZE; ++k) {
        binGains[k] = bandGains[g_barkBandOfBin[k]];
    }

    // Synthesize PCM
    SynthesizeFrame(binGains, m_pcmOut);
    return true;
}

float RNNoisePostProcess::GetVadProb() const {
    return m_vadProb;
}

