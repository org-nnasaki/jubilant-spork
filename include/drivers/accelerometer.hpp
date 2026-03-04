#pragma once

#include "sensor_base.hpp"
#include "spi_interface.hpp"

namespace sensor::drivers {

class Accelerometer : public ISensor {
public:
    explicit Accelerometer(ISPI& spi);

    Status init() override;
    Status reset() override;
    bool isConnected() override;
    uint8_t getDeviceId() override;
    std::string getName() const override;

private:
    ISPI& spi_;
};

} // namespace sensor::drivers
