// RNNoiseModel.hpp
#pragma once
#include "Model.hpp"

namespace arm {
namespace app {

class RNNoiseModel : public Model {
public:
    RNNoiseModel() = default;
    ~RNNoiseModel() override = default;

protected:
    const tflite::MicroOpResolver& GetOpResolver() override;
    bool EnlistOperations() override;

private:
    tflite::MicroMutableOpResolver<8> m_opResolver;
};

} // namespace app
} // namespace arm
