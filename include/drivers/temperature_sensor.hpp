#pragma once

#include <cstdint>
#include <string>
#include "sensor_base.hpp"
#include "i2c_interface.hpp"

namespace sensor::drivers {

class TemperatureSensor : public ISensor {
public:
    // Register addresses
    static constexpr uint8_t REG_CHIP_ID       = 0xD0;
    static constexpr uint8_t REG_RESET         = 0xE0;
    static constexpr uint8_t REG_CTRL_HUM      = 0xF2;
    static constexpr uint8_t REG_STATUS        = 0xF3;
    static constexpr uint8_t REG_CTRL_MEAS     = 0xF4;
    static constexpr uint8_t REG_CONFIG        = 0xF5;
    static constexpr uint8_t REG_DATA_START    = 0xF7;
    static constexpr uint8_t REG_CALIB_TEMP_PRESS = 0x88;
    static constexpr uint8_t REG_CALIB_HUM_H1  = 0xA1;
    static constexpr uint8_t REG_CALIB_HUM_H2  = 0xE1;

    // Constants
    static constexpr uint8_t CHIP_ID           = 0x60;
    static constexpr uint8_t RESET_VALUE       = 0xB6;
    static constexpr uint8_t DATA_LENGTH       = 8;
    static constexpr uint8_t CALIB_TP_LENGTH   = 26;
    static constexpr uint8_t CALIB_HUM_LENGTH  = 7;

    // Power modes
    static constexpr uint8_t MODE_SLEEP  = 0x00;
    static constexpr uint8_t MODE_FORCED = 0x01;
    static constexpr uint8_t MODE_NORMAL = 0x03;

    struct CalibrationData {
        // Temperature
        uint16_t dig_T1 = 0;
        int16_t  dig_T2 = 0;
        int16_t  dig_T3 = 0;
        // Pressure
        uint16_t dig_P1 = 0;
        int16_t  dig_P2 = 0;
        int16_t  dig_P3 = 0;
        int16_t  dig_P4 = 0;
        int16_t  dig_P5 = 0;
        int16_t  dig_P6 = 0;
        int16_t  dig_P7 = 0;
        int16_t  dig_P8 = 0;
        int16_t  dig_P9 = 0;
        // Humidity
        uint8_t  dig_H1 = 0;
        int16_t  dig_H2 = 0;
        uint8_t  dig_H3 = 0;
        int16_t  dig_H4 = 0;
        int16_t  dig_H5 = 0;
        int8_t   dig_H6 = 0;
    };

    explicit TemperatureSensor(II2C& i2c, uint8_t address = 0x76);

    // ISensor overrides
    Status init() override;
    Status reset() override;
    bool isConnected() override;
    uint8_t getDeviceId() override;
    std::string getName() const override;

    // Sensor-specific API
    Status readTemperature(float& tempC);
    Status readAll(float& tempC, float& pressHPa, float& humPercent);
    const CalibrationData& calibration() const;
    Status setMode(uint8_t mode);

private:
    Status readCalibration();
    Status readRawData(int32_t& rawTemp, int32_t& rawPress, int32_t& rawHum);

    // Compensation (integer arithmetic per datasheet)
    int32_t  compensateTemperature(int32_t adcT);
    uint32_t compensatePressure(int32_t adcP);
    uint32_t compensateHumidity(int32_t adcH);

    II2C& i2c_;
    uint8_t address_;
    bool initialized_ = false;
    CalibrationData cal_{};
    int32_t tFine_ = 0;
};

} // namespace sensor::drivers
