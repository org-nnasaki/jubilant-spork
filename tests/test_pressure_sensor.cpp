#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "drivers/pressure_sensor.hpp"

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

class PressureSensorTest : public ::testing::Test {
protected:
    static constexpr uint8_t ADDR = 0x76;
    MockI2C i2c_;
    PressureSensor sensor_{i2c_, ADDR};

    // Helper: expect chip ID read returning BMP280 ID
    void expectChipIdRead() {
        EXPECT_CALL(i2c_, readRegister(ADDR, PressureSensor::REG_CHIP_ID, _))
            .WillOnce(DoAll(SetArgReferee<2>(PressureSensor::CHIP_ID), Return(Status::Ok)));
    }

    // Helper: expect successful soft-reset write
    void expectReset() {
        EXPECT_CALL(i2c_, writeRegister(ADDR, PressureSensor::REG_RESET, PressureSensor::RESET_CMD))
            .WillOnce(Return(Status::Ok));
    }

    // Helper: expect calibration read with realistic BMP280 values
    void expectCalibration() {
        // dig_T1=27504, dig_T2=26435, dig_T3=-1000
        // dig_P1=36477, dig_P2=-10685, dig_P3=3024, dig_P4=2855
        // dig_P5=140, dig_P6=-7, dig_P7=15500, dig_P8=-14600, dig_P9=6000
        static const uint8_t cal[24] = {
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
        EXPECT_CALL(i2c_, readRegisters(ADDR, PressureSensor::REG_CALIB_START, _, PressureSensor::CALIB_SIZE))
            .WillOnce(DoAll(SetArrayArgument<2>(cal, cal + 24), Return(Status::Ok)));
    }

    // Helper: expect config and ctrl_meas writes during init
    void expectConfigWrites() {
        EXPECT_CALL(i2c_, writeRegister(ADDR, PressureSensor::REG_CONFIG, PressureSensor::DEFAULT_CONFIG))
            .WillOnce(Return(Status::Ok));
        EXPECT_CALL(i2c_, writeRegister(ADDR, PressureSensor::REG_CTRL_MEAS, PressureSensor::DEFAULT_CTRL_MEAS))
            .WillOnce(Return(Status::Ok));
    }

    // Full successful init sequence
    void initSuccessfully() {
        expectChipIdRead();
        expectReset();
        expectCalibration();
        expectConfigWrites();
        ASSERT_EQ(sensor_.init(), Status::Ok);
    }
};

// ─── Init tests ────────────────────────────────────────────────────────────────

TEST_F(PressureSensorTest, Init_Success) {
    initSuccessfully();
}

TEST_F(PressureSensorTest, Init_WrongChipId_ReturnsDeviceNotFound) {
    EXPECT_CALL(i2c_, readRegister(ADDR, PressureSensor::REG_CHIP_ID, _))
        .WillOnce(DoAll(SetArgReferee<2>(uint8_t{0xFF}), Return(Status::Ok)));

    EXPECT_EQ(sensor_.init(), Status::DeviceNotFound);
}

TEST_F(PressureSensorTest, Init_I2CErrorOnChipIdRead) {
    EXPECT_CALL(i2c_, readRegister(ADDR, PressureSensor::REG_CHIP_ID, _))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(sensor_.init(), Status::Error);
}

TEST_F(PressureSensorTest, Init_ResetWriteFails) {
    expectChipIdRead();
    EXPECT_CALL(i2c_, writeRegister(ADDR, PressureSensor::REG_RESET, PressureSensor::RESET_CMD))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(sensor_.init(), Status::Error);
}

TEST_F(PressureSensorTest, Init_CalibrationReadFails) {
    expectChipIdRead();
    expectReset();
    EXPECT_CALL(i2c_, readRegisters(ADDR, PressureSensor::REG_CALIB_START, _, PressureSensor::CALIB_SIZE))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(sensor_.init(), Status::Error);
}

TEST_F(PressureSensorTest, Init_ConfigWriteFails) {
    expectChipIdRead();
    expectReset();
    expectCalibration();
    EXPECT_CALL(i2c_, writeRegister(ADDR, PressureSensor::REG_CONFIG, PressureSensor::DEFAULT_CONFIG))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(sensor_.init(), Status::Error);
}

// ─── Basic API tests ──────────────────────────────────────────────────────────

TEST_F(PressureSensorTest, GetName_ReturnsBMP280) {
    EXPECT_EQ(sensor_.getName(), "BMP280");
}

TEST_F(PressureSensorTest, GetDeviceId) {
    EXPECT_CALL(i2c_, readRegister(ADDR, PressureSensor::REG_CHIP_ID, _))
        .WillOnce(DoAll(SetArgReferee<2>(PressureSensor::CHIP_ID), Return(Status::Ok)));
    EXPECT_EQ(sensor_.getDeviceId(), 0x58);
}

TEST_F(PressureSensorTest, IsConnected_True) {
    EXPECT_CALL(i2c_, readRegister(ADDR, PressureSensor::REG_CHIP_ID, _))
        .WillOnce(DoAll(SetArgReferee<2>(PressureSensor::CHIP_ID), Return(Status::Ok)));
    EXPECT_TRUE(sensor_.isConnected());
}

TEST_F(PressureSensorTest, IsConnected_False_WrongId) {
    EXPECT_CALL(i2c_, readRegister(ADDR, PressureSensor::REG_CHIP_ID, _))
        .WillOnce(DoAll(SetArgReferee<2>(uint8_t{0xAA}), Return(Status::Ok)));
    EXPECT_FALSE(sensor_.isConnected());
}

// ─── Calibration data ─────────────────────────────────────────────────────────

TEST_F(PressureSensorTest, CalibrationData_ParsedCorrectly) {
    initSuccessfully();
    const auto& cal = sensor_.calibration();

    EXPECT_EQ(cal.dig_T1, 27504);
    EXPECT_EQ(cal.dig_T2, 26435);
    EXPECT_EQ(cal.dig_T3, -1000);
    EXPECT_EQ(cal.dig_P1, 36477);
    EXPECT_EQ(cal.dig_P2, -10685);
    EXPECT_EQ(cal.dig_P3, 3024);
    EXPECT_EQ(cal.dig_P4, 2855);
    EXPECT_EQ(cal.dig_P5, 140);
    EXPECT_EQ(cal.dig_P6, -7);
    EXPECT_EQ(cal.dig_P7, 15500);
    EXPECT_EQ(cal.dig_P8, -14600);
    EXPECT_EQ(cal.dig_P9, 6000);
}

// ─── Not-initialized guards ───────────────────────────────────────────────────

TEST_F(PressureSensorTest, ReadPressure_NotInitialized) {
    float press = 0.0f;
    EXPECT_EQ(sensor_.readPressure(press), Status::NotInitialized);
}

TEST_F(PressureSensorTest, ReadTemperature_NotInitialized) {
    float temp = 0.0f;
    EXPECT_EQ(sensor_.readTemperature(temp), Status::NotInitialized);
}

TEST_F(PressureSensorTest, ReadAll_NotInitialized) {
    float temp = 0.0f, press = 0.0f;
    EXPECT_EQ(sensor_.readAll(temp, press), Status::NotInitialized);
}

TEST_F(PressureSensorTest, SetMode_NotInitialized) {
    EXPECT_EQ(sensor_.setMode(PressureSensor::MODE_SLEEP), Status::NotInitialized);
}

// ─── ReadTemperature ───────────────────────────────────────────────────────────

TEST_F(PressureSensorTest, ReadTemperature_Success) {
    initSuccessfully();

    // press_raw = (0x65<<12)|(0x5A<<4)|(0xC0>>4) = 415148
    // temp_raw  = (0x7E<<12)|(0xF5<<4)|(0x00>>4) = 520016
    static const uint8_t data[6] = {
        0x65, 0x5A, 0xC0,  // press
        0x7E, 0xF5, 0x00,  // temp
    };
    EXPECT_CALL(i2c_, readRegisters(ADDR, PressureSensor::REG_DATA_START, _, PressureSensor::DATA_SIZE))
        .WillOnce(DoAll(SetArrayArgument<2>(data, data + 6), Return(Status::Ok)));

    float temp = 0.0f;
    EXPECT_EQ(sensor_.readTemperature(temp), Status::Ok);
    EXPECT_GT(temp, -40.0f);
    EXPECT_LT(temp, 85.0f);
}

TEST_F(PressureSensorTest, ReadTemperature_I2CError) {
    initSuccessfully();

    EXPECT_CALL(i2c_, readRegisters(ADDR, PressureSensor::REG_DATA_START, _, PressureSensor::DATA_SIZE))
        .WillOnce(Return(Status::Error));

    float temp = 0.0f;
    EXPECT_EQ(sensor_.readTemperature(temp), Status::Error);
}

// ─── ReadPressure ──────────────────────────────────────────────────────────────

TEST_F(PressureSensorTest, ReadPressure_Success) {
    initSuccessfully();

    static const uint8_t data[6] = {
        0x65, 0x5A, 0xC0,  // press
        0x7E, 0xF5, 0x00,  // temp
    };
    EXPECT_CALL(i2c_, readRegisters(ADDR, PressureSensor::REG_DATA_START, _, PressureSensor::DATA_SIZE))
        .WillOnce(DoAll(SetArrayArgument<2>(data, data + 6), Return(Status::Ok)));

    float press = 0.0f;
    EXPECT_EQ(sensor_.readPressure(press), Status::Ok);
    EXPECT_GT(press, 300.0f);
    EXPECT_LT(press, 1100.0f);
}

TEST_F(PressureSensorTest, ReadPressure_I2CError) {
    initSuccessfully();

    EXPECT_CALL(i2c_, readRegisters(ADDR, PressureSensor::REG_DATA_START, _, PressureSensor::DATA_SIZE))
        .WillOnce(Return(Status::Error));

    float press = 0.0f;
    EXPECT_EQ(sensor_.readPressure(press), Status::Error);
}

// ─── ReadAll ───────────────────────────────────────────────────────────────────

TEST_F(PressureSensorTest, ReadAll_Success) {
    initSuccessfully();

    static const uint8_t data[6] = {
        0x65, 0x5A, 0xC0,
        0x7E, 0xF5, 0x00,
    };
    EXPECT_CALL(i2c_, readRegisters(ADDR, PressureSensor::REG_DATA_START, _, PressureSensor::DATA_SIZE))
        .WillOnce(DoAll(SetArrayArgument<2>(data, data + 6), Return(Status::Ok)));

    float temp = 0.0f, press = 0.0f;
    EXPECT_EQ(sensor_.readAll(temp, press), Status::Ok);
    EXPECT_GT(temp, -40.0f);
    EXPECT_LT(temp, 85.0f);
    EXPECT_GT(press, 300.0f);
    EXPECT_LT(press, 1100.0f);
}

// ─── SetMode ───────────────────────────────────────────────────────────────────

TEST_F(PressureSensorTest, SetMode_Success) {
    initSuccessfully();

    // Read current ctrl_meas (0x27), write with sleep mode bits cleared
    EXPECT_CALL(i2c_, readRegister(ADDR, PressureSensor::REG_CTRL_MEAS, _))
        .WillOnce(DoAll(SetArgReferee<2>(uint8_t{0x27}), Return(Status::Ok)));
    EXPECT_CALL(i2c_, writeRegister(ADDR, PressureSensor::REG_CTRL_MEAS, 0x24))
        .WillOnce(Return(Status::Ok));

    EXPECT_EQ(sensor_.setMode(PressureSensor::MODE_SLEEP), Status::Ok);
}

TEST_F(PressureSensorTest, SetMode_Forced2_Success) {
    initSuccessfully();

    // Read current ctrl_meas (0x27), mask out mode bits and OR in MODE_FORCED2 (0x02): (0x27 & 0xFC) | 0x02 = 0x26
    EXPECT_CALL(i2c_, readRegister(ADDR, PressureSensor::REG_CTRL_MEAS, _))
        .WillOnce(DoAll(SetArgReferee<2>(uint8_t{0x27}), Return(Status::Ok)));
    EXPECT_CALL(i2c_, writeRegister(ADDR, PressureSensor::REG_CTRL_MEAS, 0x26))
        .WillOnce(Return(Status::Ok));

    EXPECT_EQ(sensor_.setMode(PressureSensor::MODE_FORCED2), Status::Ok);
}

TEST_F(PressureSensorTest, SetMode_InvalidParam) {
    initSuccessfully();
    EXPECT_EQ(sensor_.setMode(0x04), Status::InvalidParam);
}

// ─── Reset ─────────────────────────────────────────────────────────────────────

TEST_F(PressureSensorTest, Reset_Success) {
    EXPECT_CALL(i2c_, writeRegister(ADDR, PressureSensor::REG_RESET, PressureSensor::RESET_CMD))
        .WillOnce(Return(Status::Ok));
    EXPECT_EQ(sensor_.reset(), Status::Ok);
}

TEST_F(PressureSensorTest, Reset_ClearsInitialized) {
    initSuccessfully();

    EXPECT_CALL(i2c_, writeRegister(ADDR, PressureSensor::REG_RESET, PressureSensor::RESET_CMD))
        .WillOnce(Return(Status::Ok));
    EXPECT_EQ(sensor_.reset(), Status::Ok);

    // After reset, reads should fail with NotInitialized
    float temp = 0.0f;
    EXPECT_EQ(sensor_.readTemperature(temp), Status::NotInitialized);
}

// ─── Alternate address ─────────────────────────────────────────────────────────

TEST_F(PressureSensorTest, AlternateAddress_0x77) {
    MockI2C i2c2;
    PressureSensor alt(i2c2, 0x77);

    EXPECT_CALL(i2c2, readRegister(0x77, PressureSensor::REG_CHIP_ID, _))
        .WillOnce(DoAll(SetArgReferee<2>(PressureSensor::CHIP_ID), Return(Status::Ok)));
    EXPECT_TRUE(alt.isConnected());
}
