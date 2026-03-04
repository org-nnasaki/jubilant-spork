#pragma once

#include <cstdint>
#include <string>

#include "sensor_base.hpp"
#include "spi_interface.hpp"

namespace sensor::drivers {

class Accelerometer : public ISensor {
public:
    // ADXL345 register addresses
    static constexpr uint8_t REG_DEVID       = 0x00;
    static constexpr uint8_t REG_OFSX        = 0x1E;
    static constexpr uint8_t REG_OFSY        = 0x1F;
    static constexpr uint8_t REG_OFSZ        = 0x20;
    static constexpr uint8_t REG_BW_RATE     = 0x2C;
    static constexpr uint8_t REG_POWER_CTL   = 0x2D;
    static constexpr uint8_t REG_INT_ENABLE  = 0x2E;
    static constexpr uint8_t REG_INT_MAP     = 0x2F;
    static constexpr uint8_t REG_INT_SOURCE  = 0x30;
    static constexpr uint8_t REG_DATA_FORMAT = 0x31;
    static constexpr uint8_t REG_DATAX0      = 0x32;
    static constexpr uint8_t REG_FIFO_CTL    = 0x38;
    static constexpr uint8_t REG_FIFO_STATUS = 0x39;

    static constexpr uint8_t CHIP_ID = 0xE5;

    // DATA_FORMAT bit masks
    static constexpr uint8_t DATA_FORMAT_FULL_RES  = 0x08;
    static constexpr uint8_t DATA_FORMAT_RANGE_MASK = 0x03;

    // POWER_CTL bit masks
    static constexpr uint8_t POWER_CTL_MEASURE = 0x08;
    static constexpr uint8_t POWER_CTL_SLEEP   = 0x04;

    // BW_RATE bit masks
    static constexpr uint8_t BW_RATE_LOW_POWER = 0x10;
    static constexpr uint8_t BW_RATE_RATE_MASK = 0x0F;

    // Full-resolution scale factor: 4 mg/LSB
    static constexpr float FULL_RES_SCALE = 0.004f;

    // Acceleration data size in bytes
    static constexpr uint8_t ACCEL_DATA_SIZE = 6;

    enum class Range : uint8_t {
        G2  = 0x00,
        G4  = 0x01,
        G8  = 0x02,
        G16 = 0x03
    };

    enum class DataRate : uint8_t {
        Hz0_10   = 0x00,
        Hz0_20   = 0x01,
        Hz0_39   = 0x02,
        Hz0_78   = 0x03,
        Hz1_56   = 0x04,
        Hz3_13   = 0x05,
        Hz6_25   = 0x06,
        Hz12_5   = 0x07,
        Hz25     = 0x08,
        Hz50     = 0x09,
        Hz100    = 0x0A,
        Hz200    = 0x0B,
        Hz400    = 0x0C,
        Hz800    = 0x0D,
        Hz1600   = 0x0E,
        Hz3200   = 0x0F
    };

    enum class FifoMode : uint8_t {
        Bypass  = 0x00,
        Fifo    = 0x01,
        Stream  = 0x02,
        Trigger = 0x03
    };

    explicit Accelerometer(ISPI& spi);

    // ISensor interface
    Status init() override;
    Status reset() override;
    bool isConnected() override;
    uint8_t getDeviceId() override;
    std::string getName() const override;

    // Acceleration reading
    Status readAcceleration(float& x, float& y, float& z);
    Status readRawAcceleration(int16_t& x, int16_t& y, int16_t& z);

    // Configuration
    Status setRange(Range range);
    Status setDataRate(DataRate rate);
    Status setMeasureMode(bool enable);
    Range getRange() const;

private:
    Status readRegister(uint8_t reg, uint8_t& value);
    Status writeRegister(uint8_t reg, uint8_t value);
    float scaleFactorForRange(Range range) const;

    ISPI& spi_;
    bool initialized_{false};
    bool fullResolution_{true};
    Range range_{Range::G2};
};

} // namespace sensor::drivers
