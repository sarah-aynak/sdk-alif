// RNNoiseModel.cc
#include "RNNoiseModel.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(RNNoiseModel);

const tflite::MicroOpResolver& arm::app::RNNoiseModel::GetOpResolver() {
    return this->m_opResolver;
}

bool arm::app::RNNoiseModel::EnlistOperations() {
#ifndef ETHOS_U_NPU_ASSUMED
    m_opResolver.AddFullyConnected();
    m_opResolver.AddMul();
    m_opResolver.AddAdd();
    m_opResolver.AddTanh();
    m_opResolver.AddLogistic();   // sigmoid
    m_opResolver.AddReshape();
    m_opResolver.AddSplit();
    m_opResolver.AddSplitV();
    m_opResolver.AddPack();
    m_opResolver.AddUnpack();
    m_opResolver.AddConcatenation();  // for ConcatTFLite
    m_opResolver.AddSub();
    m_opResolver.AddQuantize();
    m_opResolver.AddDequantize();
#endif

    if (kTfLiteOk == m_opResolver.AddEthosU()) {
        //LOG_INF("Added %s support to op resolver", tflite::GetString_ETHOSU());
    } else {
        //LOG_WRN("Ethos-U not available, running on CPU only");
    }
    return true;
}

// ✅ NEW: Forward Init() to base class
bool arm::app::RNNoiseModel::Init(uint8_t* tensorArenaAddr, uint32_t tensorArenaSize,
                                   const uint8_t* nnModelAddr, uint32_t nnModelSize) {
    return Model::Init(tensorArenaAddr, tensorArenaSize, nnModelAddr, nnModelSize);
}
