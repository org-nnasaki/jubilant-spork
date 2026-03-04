#pragma once

#include "sensor_base.hpp"
#include "i2c_interface.hpp"

namespace sensor::drivers {

class PressureSensor : public ISensor {
public:
    explicit PressureSensor(II2C& i2c, uint8_t address = 0x76);

    Status init() override;
    Status reset() override;
    bool isConnected() override;
    uint8_t getDeviceId() override;
    std::string getName() const override;

private:
    II2C& i2c_;
    uint8_t address_;
};

} // namespace sensor::drivers
