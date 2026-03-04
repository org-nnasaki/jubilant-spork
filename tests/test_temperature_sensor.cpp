#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "drivers/temperature_sensor.hpp"

using namespace sensor;
using namespace sensor::drivers;
using ::testing::_;
using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::SetArrayArgument;

// ─── Mock I2C bus ──────────────────────────────────────────────────────────────

class MockI2C : public II2C {
public:
    MOCK_METHOD(Status, readRegister,  (uint8_t, uint8_t, uint8_t&),       (override));
    MOCK_METHOD(Status, writeRegister, (uint8_t, uint8_t, uint8_t),        (override));
    MOCK_METHOD(Status, readRegisters, (uint8_t, uint8_t, uint8_t*, uint8_t), (override));
    MOCK_METHOD(Status, writeRegisters,(uint8_t, uint8_t, const uint8_t*, uint8_t), (override));
};

// ─── Fixture ───────────────────────────────────────────────────────────────────

class TemperatureSensorTest : public ::testing::Test {
protected:
    static constexpr uint8_t ADDR = 0x76;
    MockI2C i2c_;
    TemperatureSensor sensor_{i2c_, ADDR};

    // Helper: expect chip ID read returning BME280 ID
    void expectChipIdRead() {
        EXPECT_CALL(i2c_, readRegister(ADDR, TemperatureSensor::REG_CHIP_ID, _))
            .WillOnce(DoAll(SetArgReferee<2>(TemperatureSensor::CHIP_ID), Return(Status::Ok)));
    }

    // Helper: expect calibration reads with realistic BME280 values
    void expectCalibration() {
        // Temp/Press calibration block (26 bytes at 0x88)
        // dig_T1=27504, dig_T2=26435, dig_T3=-1000
        // dig_P1=36477, dig_P2=-10685, dig_P3=3024, dig_P4=2855,
        // dig_P5=140, dig_P6=-7, dig_P7=15500, dig_P8=-14600, dig_P9=6000
        static const uint8_t tpCal[26] = {
            0x70, 0x6B,  // dig_T1 = 27504
            0x43, 0x67,  // dig_T2 = 26435
            0x18, 0xFC,  // dig_T3 = -1000
            0x7D, 0x8E,  // dig_P1 = 36477
            0x43, 0xD6,  // dig_P2 = -10685
            0xD0, 0x0B,  // dig_P3 = 3024
            0x27, 0x0B,  // dig_P4 = 2855
            0x8C, 0x00,  // dig_P5 = 140
            0xF9, 0xFF,  // dig_P6 = -7
            0x8C, 0x3C,  // dig_P7 = 15500
            0xF8, 0xC6,  // dig_P8 = -14600
            0x70, 0x17,  // dig_P9 = 6000
        };
        EXPECT_CALL(i2c_, readRegisters(ADDR, TemperatureSensor::REG_CALIB_TEMP_PRESS, _, 26))
            .WillOnce(DoAll(SetArrayArgument<2>(tpCal, tpCal + 26), Return(Status::Ok)));

        // Humidity H1 (0xA1): dig_H1 = 75
        EXPECT_CALL(i2c_, readRegister(ADDR, TemperatureSensor::REG_CALIB_HUM_H1, _))
            .WillOnce(DoAll(SetArgReferee<2>(uint8_t{75}), Return(Status::Ok)));

        // Humidity H2..H6 (0xE1, 7 bytes)
        // dig_H2=370, dig_H3=0, dig_H4=306, dig_H5=57, dig_H6=30
        static const uint8_t hCal[7] = {
            0x72, 0x01,  // dig_H2 = 370
            0x00,        // dig_H3 = 0
            0x13,        // dig_H4 high byte (0x13 → 19)
            0x92,        // 0xE5: low nibble for H4 (0x2), high nibble for H5 (0x9 → 9)
            0x03,        // dig_H5 high byte (0x03 → 3)  → H5 = (3<<4)|(9) = 57
            0x1E,        // dig_H6 = 30
        };
        // Recalculate: dig_H4 = (0x13 << 4) | (0x92 & 0x0F) = (19<<4)|2 = 306
        // dig_H5 = (0x03 << 4) | (0x92 >> 4) = 48 | 9 = 57
        EXPECT_CALL(i2c_, readRegisters(ADDR, TemperatureSensor::REG_CALIB_HUM_H2, _, 7))
            .WillOnce(DoAll(SetArrayArgument<2>(hCal, hCal + 7), Return(Status::Ok)));
    }

    // Helper: expect config writes during init
    void expectConfigWrites() {
        EXPECT_CALL(i2c_, writeRegister(ADDR, TemperatureSensor::REG_CTRL_HUM, 0x01))
            .WillOnce(Return(Status::Ok));
        EXPECT_CALL(i2c_, writeRegister(ADDR, TemperatureSensor::REG_CTRL_MEAS, 0x27))
            .WillOnce(Return(Status::Ok));
        EXPECT_CALL(i2c_, writeRegister(ADDR, TemperatureSensor::REG_CONFIG, 0x00))
            .WillOnce(Return(Status::Ok));
    }

    // Full successful init sequence
    void initSuccessfully() {
        expectChipIdRead();
        expectCalibration();
        expectConfigWrites();
        ASSERT_EQ(sensor_.init(), Status::Ok);
    }
};

// ─── Init tests ────────────────────────────────────────────────────────────────

TEST_F(TemperatureSensorTest, Init_Success) {
    initSuccessfully();
}

TEST_F(TemperatureSensorTest, Init_DeviceNotFound_WrongChipId) {
    EXPECT_CALL(i2c_, readRegister(ADDR, TemperatureSensor::REG_CHIP_ID, _))
        .WillOnce(DoAll(SetArgReferee<2>(uint8_t{0xFF}), Return(Status::Ok)));

    EXPECT_EQ(sensor_.init(), Status::DeviceNotFound);
}

TEST_F(TemperatureSensorTest, Init_DeviceNotFound_I2CError) {
    EXPECT_CALL(i2c_, readRegister(ADDR, TemperatureSensor::REG_CHIP_ID, _))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(sensor_.init(), Status::DeviceNotFound);
}

TEST_F(TemperatureSensorTest, Init_CalibrationReadFails) {
    expectChipIdRead();
    EXPECT_CALL(i2c_, readRegisters(ADDR, TemperatureSensor::REG_CALIB_TEMP_PRESS, _, 26))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(sensor_.init(), Status::Error);
}

// ─── Basic API tests ──────────────────────────────────────────────────────────

TEST_F(TemperatureSensorTest, GetName) {
    EXPECT_EQ(sensor_.getName(), "BME280");
}

TEST_F(TemperatureSensorTest, GetDeviceId) {
    EXPECT_CALL(i2c_, readRegister(ADDR, TemperatureSensor::REG_CHIP_ID, _))
        .WillOnce(DoAll(SetArgReferee<2>(TemperatureSensor::CHIP_ID), Return(Status::Ok)));
    EXPECT_EQ(sensor_.getDeviceId(), 0x60);
}

TEST_F(TemperatureSensorTest, IsConnected_True) {
    EXPECT_CALL(i2c_, readRegister(ADDR, TemperatureSensor::REG_CHIP_ID, _))
        .WillOnce(DoAll(SetArgReferee<2>(TemperatureSensor::CHIP_ID), Return(Status::Ok)));
    EXPECT_TRUE(sensor_.isConnected());
}

TEST_F(TemperatureSensorTest, IsConnected_False_WrongId) {
    EXPECT_CALL(i2c_, readRegister(ADDR, TemperatureSensor::REG_CHIP_ID, _))
        .WillOnce(DoAll(SetArgReferee<2>(uint8_t{0xAA}), Return(Status::Ok)));
    EXPECT_FALSE(sensor_.isConnected());
}

TEST_F(TemperatureSensorTest, Reset_Success) {
    EXPECT_CALL(i2c_, writeRegister(ADDR, TemperatureSensor::REG_RESET, TemperatureSensor::RESET_VALUE))
        .WillOnce(Return(Status::Ok));
    EXPECT_EQ(sensor_.reset(), Status::Ok);
}

TEST_F(TemperatureSensorTest, CalibrationData_ParsedCorrectly) {
    initSuccessfully();
    const auto& cal = sensor_.calibration();

    EXPECT_EQ(cal.dig_T1, 27504);
    EXPECT_EQ(cal.dig_T2, 26435);
    EXPECT_EQ(cal.dig_T3, -1000);
    EXPECT_EQ(cal.dig_P1, 36477);
    EXPECT_EQ(cal.dig_H1, 75);
    EXPECT_EQ(cal.dig_H2, 370);
    EXPECT_EQ(cal.dig_H4, 306);  // (0x13<<4)|(0x92&0x0F) = 304+2
    EXPECT_EQ(cal.dig_H5, 57);   // (0x03<<4)|(0x92>>4) = 48+9
    EXPECT_EQ(cal.dig_H6, 30);
}

// ─── Not-initialized guard ─────────────────────────────────────────────────────

TEST_F(TemperatureSensorTest, ReadTemperature_NotInitialized) {
    float temp = 0.0f;
    EXPECT_EQ(sensor_.readTemperature(temp), Status::NotInitialized);
}

TEST_F(TemperatureSensorTest, ReadAll_NotInitialized) {
    float t = 0, p = 0, h = 0;
    EXPECT_EQ(sensor_.readAll(t, p, h), Status::NotInitialized);
}

TEST_F(TemperatureSensorTest, SetMode_NotInitialized) {
    EXPECT_EQ(sensor_.setMode(TemperatureSensor::MODE_SLEEP), Status::NotInitialized);
}

// ─── ReadTemperature ───────────────────────────────────────────────────────────

TEST_F(TemperatureSensorTest, ReadTemperature_Success) {
    initSuccessfully();

    // Raw data: adc_T = 519888 (0x7EF50) → approx 25°C with our cal values
    // Data registers F7..FE: press(3), temp(3), hum(2)
    static const uint8_t data[8] = {
        0x50, 0x00, 0x00,  // press_raw = 0x500000>>4 = don't care
        0x7E, 0xF5, 0x00,  // temp_raw = (0x7E<<12)|(0xF5<<4)|(0x00>>4) = 519888
        0x80, 0x00,        // hum_raw = 32768
    };
    EXPECT_CALL(i2c_, readRegisters(ADDR, TemperatureSensor::REG_DATA_START, _, 8))
        .WillOnce(DoAll(SetArrayArgument<2>(data, data + 8), Return(Status::Ok)));

    float temp = 0.0f;
    EXPECT_EQ(sensor_.readTemperature(temp), Status::Ok);
    // With these calibration values, result should be a reasonable temperature
    EXPECT_GT(temp, -40.0f);
    EXPECT_LT(temp, 85.0f);
}

TEST_F(TemperatureSensorTest, ReadTemperature_I2CError) {
    initSuccessfully();

    EXPECT_CALL(i2c_, readRegisters(ADDR, TemperatureSensor::REG_DATA_START, _, 8))
        .WillOnce(Return(Status::Error));

    float temp = 0.0f;
    EXPECT_EQ(sensor_.readTemperature(temp), Status::Error);
}

// ─── ReadAll ───────────────────────────────────────────────────────────────────

TEST_F(TemperatureSensorTest, ReadAll_Success) {
    initSuccessfully();

    static const uint8_t data[8] = {
        0x50, 0x00, 0x00,  // press
        0x7E, 0xF5, 0x00,  // temp
        0x80, 0x00,        // hum
    };
    EXPECT_CALL(i2c_, readRegisters(ADDR, TemperatureSensor::REG_DATA_START, _, 8))
        .WillOnce(DoAll(SetArrayArgument<2>(data, data + 8), Return(Status::Ok)));

    float temp = 0, press = 0, hum = 0;
    EXPECT_EQ(sensor_.readAll(temp, press, hum), Status::Ok);

    // Sanity checks on ranges
    EXPECT_GT(temp, -40.0f);
    EXPECT_LT(temp, 85.0f);
    EXPECT_GE(press, 0.0f);
    EXPECT_GE(hum, 0.0f);
    EXPECT_LE(hum, 100.0f);
}

// ─── SetMode ───────────────────────────────────────────────────────────────────

TEST_F(TemperatureSensorTest, SetMode_Success) {
    initSuccessfully();

    // Read current ctrl_meas, then write with new mode bits
    EXPECT_CALL(i2c_, readRegister(ADDR, TemperatureSensor::REG_CTRL_MEAS, _))
        .WillOnce(DoAll(SetArgReferee<2>(uint8_t{0x27}), Return(Status::Ok)));
    EXPECT_CALL(i2c_, writeRegister(ADDR, TemperatureSensor::REG_CTRL_MEAS, 0x24))
        .WillOnce(Return(Status::Ok));

    EXPECT_EQ(sensor_.setMode(TemperatureSensor::MODE_SLEEP), Status::Ok);
}

TEST_F(TemperatureSensorTest, SetMode_InvalidParam) {
    initSuccessfully();
    EXPECT_EQ(sensor_.setMode(0x04), Status::InvalidParam);
}

// ─── Alternate address ─────────────────────────────────────────────────────────

TEST_F(TemperatureSensorTest, AlternateAddress_0x77) {
    MockI2C i2c2;
    TemperatureSensor alt(i2c2, 0x77);

    EXPECT_CALL(i2c2, readRegister(0x77, TemperatureSensor::REG_CHIP_ID, _))
        .WillOnce(DoAll(SetArgReferee<2>(TemperatureSensor::CHIP_ID), Return(Status::Ok)));
    EXPECT_TRUE(alt.isConnected());
}

// ─── Reset clears initialized state ───────────────────────────────────────────

TEST_F(TemperatureSensorTest, Reset_ClearsInitialized) {
    initSuccessfully();

    EXPECT_CALL(i2c_, writeRegister(ADDR, TemperatureSensor::REG_RESET, TemperatureSensor::RESET_VALUE))
        .WillOnce(Return(Status::Ok));
    EXPECT_EQ(sensor_.reset(), Status::Ok);

    // After reset, reads should fail with NotInitialized
    float temp = 0;
    EXPECT_EQ(sensor_.readTemperature(temp), Status::NotInitialized);
}

// ─── ctrl_hum ordering ────────────────────────────────────────────────────────

TEST_F(TemperatureSensorTest, Init_CtrlHumWrittenBeforeCtrlMeas) {
    expectChipIdRead();
    expectCalibration();

    // BME280 datasheet requires ctrl_hum to be written before ctrl_meas
    // for humidity oversampling changes to take effect
    {
        ::testing::InSequence seq;
        EXPECT_CALL(i2c_, writeRegister(ADDR, TemperatureSensor::REG_CTRL_HUM, 0x01))
            .WillOnce(Return(Status::Ok));
        EXPECT_CALL(i2c_, writeRegister(ADDR, TemperatureSensor::REG_CTRL_MEAS, 0x27))
            .WillOnce(Return(Status::Ok));
        EXPECT_CALL(i2c_, writeRegister(ADDR, TemperatureSensor::REG_CONFIG, 0x00))
            .WillOnce(Return(Status::Ok));
    }

    EXPECT_EQ(sensor_.init(), Status::Ok);
}

// ─── Double init ──────────────────────────────────────────────────────────────

TEST_F(TemperatureSensorTest, Init_CalledTwice_Succeeds) {
    initSuccessfully();

    // Second init should also succeed (idempotent)
    expectChipIdRead();
    expectCalibration();
    expectConfigWrites();
    EXPECT_EQ(sensor_.init(), Status::Ok);

    // Sensor should still work after re-init
    static const uint8_t data[8] = {
        0x50, 0x00, 0x00,
        0x7E, 0xF5, 0x00,
        0x80, 0x00,
    };
    EXPECT_CALL(i2c_, readRegisters(ADDR, TemperatureSensor::REG_DATA_START, _, 8))
        .WillOnce(DoAll(SetArrayArgument<2>(data, data + 8), Return(Status::Ok)));

    float temp = 0.0f;
    EXPECT_EQ(sensor_.readTemperature(temp), Status::Ok);
    EXPECT_GT(temp, -40.0f);
    EXPECT_LT(temp, 85.0f);
}
