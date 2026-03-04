#pragma once

#include <cstdint>
#include <vector>
#include "sensor_base.hpp"

namespace sensor {

class II2C {
public:
    virtual ~II2C() = default;

    virtual Status readRegister(uint8_t devAddr, uint8_t regAddr, uint8_t& value) = 0;
    virtual Status writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value) = 0;
    virtual Status readRegisters(uint8_t devAddr, uint8_t regAddr, uint8_t* buffer, uint8_t length) = 0;
    virtual Status writeRegisters(uint8_t devAddr, uint8_t regAddr, const uint8_t* buffer, uint8_t length) = 0;
};

} // namespace sensor
