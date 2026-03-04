#pragma once

#include <cstdint>
#include "sensor_base.hpp"

namespace sensor {

class ISPI {
public:
    virtual ~ISPI() = default;

    virtual Status readRegister(uint8_t regAddr, uint8_t& value) = 0;
    virtual Status writeRegister(uint8_t regAddr, uint8_t value) = 0;
    virtual Status readRegisters(uint8_t regAddr, uint8_t* buffer, uint8_t length) = 0;
    virtual Status writeRegisters(uint8_t regAddr, const uint8_t* buffer, uint8_t length) = 0;
};

} // namespace sensor
