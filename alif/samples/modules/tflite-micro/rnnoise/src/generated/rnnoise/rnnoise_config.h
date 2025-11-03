#ifndef RNNOISE_CONFIG_H
#define RNNOISE_CONFIG_H

// RNNoise parameters
#define FRAME_SIZE       480   // Samples per frame
#define NB_FEATURES      42   // Input features per frame
#define NB_BANDS         22   // Output gains per frame
#define FREQ_SIZE   (FRAME_SIZE + 1)
#define SAMPLE_RATE 48000
// GRU state sizes (from your model description)
#define VAD_GRU_SIZE     (24 * sizeof(int8_t))
#define NOISE_GRU_SIZE   (48 * sizeof(int8_t))
#define DENOISE_GRU_SIZE (96 * sizeof(int8_t))

#endif // RNNOISE_CONFIG_H
