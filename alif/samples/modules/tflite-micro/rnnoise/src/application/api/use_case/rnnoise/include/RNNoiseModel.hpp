// RNNoiseModel.hpp
#pragma once
#include "Model.hpp"

namespace arm {
namespace app {

class RNNoiseModel : public Model {
public:
    RNNoiseModel() = default;
    ~RNNoiseModel() override = default;

    bool Init(uint8_t* tensorArenaAddr, uint32_t tensorArenaSize,
          const uint8_t* nnModelAddr, uint32_t nnModelSize);

protected:
    const tflite::MicroOpResolver& GetOpResolver() override;
    bool EnlistOperations() override;

private:
    tflite::MicroMutableOpResolver<16> m_opResolver;
};

} // namespace app
} // namespace arm
