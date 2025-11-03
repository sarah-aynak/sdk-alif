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
#endif

    if (kTfLiteOk == m_opResolver.AddEthosU()) {
        LOG_INF("Added %s support to op resolver", tflite::GetString_ETHOSU());
    } else {
        LOG_WRN("Ethos-U not available, running on CPU only");
    }
    return true;
}
