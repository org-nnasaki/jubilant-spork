#pragma once

#include <cstdint>
#include <string>

#include "sensor_base.hpp"
#include "i2c_interface.hpp"

namespace sensor::drivers {

class PressureSensor : public ISensor {
public:
    // Register addresses
    static constexpr uint8_t REG_CHIP_ID    = 0xD0;
    static constexpr uint8_t REG_RESET      = 0xE0;
    static constexpr uint8_t REG_STATUS     = 0xF3;
    static constexpr uint8_t REG_CTRL_MEAS  = 0xF4;
    static constexpr uint8_t REG_CONFIG     = 0xF5;
    static constexpr uint8_t REG_DATA_START = 0xF7;
    static constexpr uint8_t REG_CALIB_START = 0x88;

    // Constants
    static constexpr uint8_t CHIP_ID        = 0x58;
    static constexpr uint8_t RESET_CMD      = 0xB6;
    static constexpr uint8_t CALIB_SIZE     = 24;
    static constexpr uint8_t DATA_SIZE      = 6;

    // Power modes
    static constexpr uint8_t MODE_SLEEP     = 0x00;
    static constexpr uint8_t MODE_FORCED    = 0x01;
    static constexpr uint8_t MODE_FORCED2   = 0x02;
    static constexpr uint8_t MODE_NORMAL    = 0x03;

    // Default ctrl_meas: osrs_t=×1 (001), osrs_p=×1 (001), mode=normal (11)
    static constexpr uint8_t DEFAULT_CTRL_MEAS = 0x27;
    // Default config: t_sb=0.5ms (000), filter=off (000), spi3w_en=0
    static constexpr uint8_t DEFAULT_CONFIG    = 0x00;

    struct CalibrationData {
        uint16_t dig_T1 = 0;
        int16_t  dig_T2 = 0;
        int16_t  dig_T3 = 0;
        uint16_t dig_P1 = 0;
        int16_t  dig_P2 = 0;
        int16_t  dig_P3 = 0;
        int16_t  dig_P4 = 0;
        int16_t  dig_P5 = 0;
        int16_t  dig_P6 = 0;
        int16_t  dig_P7 = 0;
        int16_t  dig_P8 = 0;
        int16_t  dig_P9 = 0;
    };

    explicit PressureSensor(II2C& i2c, uint8_t address = 0x76);

    Status init() override;
    Status reset() override;
    bool isConnected() override;
    uint8_t getDeviceId() override;
    std::string getName() const override;

    Status readPressure(float& pressHPa);
    Status readTemperature(float& tempC);
    Status readAll(float& tempC, float& pressHPa);
    const CalibrationData& calibration() const;
    Status setMode(uint8_t mode);

private:
    Status readCalibration();
    Status readRawData(int32_t& rawTemp, int32_t& rawPress);
    int32_t compensateTemperature(int32_t adcT);
    uint32_t compensatePressure(int32_t adcP);

    II2C& i2c_;
    uint8_t address_;
    bool initialized_ = false;
    CalibrationData calib_{};
    int32_t tFine_ = 0;
};

} // namespace sensor::drivers
