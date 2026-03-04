#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "drivers/accelerometer.hpp"

using namespace sensor;
using namespace sensor::drivers;
using ::testing::_;
using ::testing::Return;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::SetArrayArgument;

// ─── Mock SPI bus ──────────────────────────────────────────────────────────────

class MockSPI : public ISPI {
public:
    MOCK_METHOD(Status, readRegister,   (uint8_t, uint8_t&),              (override));
    MOCK_METHOD(Status, writeRegister,  (uint8_t, uint8_t),               (override));
    MOCK_METHOD(Status, readRegisters,  (uint8_t, uint8_t*, uint8_t),     (override));
    MOCK_METHOD(Status, writeRegisters, (uint8_t, const uint8_t*, uint8_t), (override));
};

// ─── Fixture ───────────────────────────────────────────────────────────────────

class AccelerometerTest : public ::testing::Test {
protected:
    MockSPI spi_;
    Accelerometer accel_{spi_};

    // Helper: expect chip ID read returning ADXL345 ID (0xE5)
    void expectChipIdRead() {
        EXPECT_CALL(spi_, readRegister(Accelerometer::REG_DEVID, _))
            .WillOnce(DoAll(SetArgReferee<1>(Accelerometer::CHIP_ID), Return(Status::Ok)));
    }

    // Helper: expect the three configuration writes during init
    void expectInitWrites() {
        // DATA_FORMAT: FULL_RES | G2 = 0x08 | 0x00 = 0x08
        EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_DATA_FORMAT, uint8_t{0x08}))
            .WillOnce(Return(Status::Ok));
        // BW_RATE: 100 Hz = 0x0A
        EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_BW_RATE, uint8_t{0x0A}))
            .WillOnce(Return(Status::Ok));
        // POWER_CTL: Measure = 0x08
        EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_POWER_CTL, Accelerometer::POWER_CTL_MEASURE))
            .WillOnce(Return(Status::Ok));
    }

    // Full successful init sequence
    void initSuccessfully() {
        expectChipIdRead();
        expectInitWrites();
        ASSERT_EQ(accel_.init(), Status::Ok);
    }
};

// ─── Init tests ────────────────────────────────────────────────────────────────

TEST_F(AccelerometerTest, Init_Success) {
    initSuccessfully();
}

TEST_F(AccelerometerTest, Init_DeviceNotFound_WrongChipId) {
    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_DEVID, _))
        .WillOnce(DoAll(SetArgReferee<1>(uint8_t{0xFF}), Return(Status::Ok)));

    EXPECT_EQ(accel_.init(), Status::DeviceNotFound);
}

TEST_F(AccelerometerTest, Init_SpiErrorOnChipIdRead) {
    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_DEVID, _))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(accel_.init(), Status::Error);
}

TEST_F(AccelerometerTest, Init_SpiErrorOnDataFormatWrite) {
    expectChipIdRead();
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_DATA_FORMAT, _))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(accel_.init(), Status::Error);
}

TEST_F(AccelerometerTest, Init_SpiErrorOnBwRateWrite) {
    expectChipIdRead();
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_DATA_FORMAT, uint8_t{0x08}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_BW_RATE, _))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(accel_.init(), Status::Error);
}

TEST_F(AccelerometerTest, Init_SpiErrorOnPowerCtlWrite) {
    expectChipIdRead();
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_DATA_FORMAT, uint8_t{0x08}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_BW_RATE, uint8_t{0x0A}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_POWER_CTL, _))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(accel_.init(), Status::Error);
}

// ─── Basic API tests ──────────────────────────────────────────────────────────

TEST_F(AccelerometerTest, GetName) {
    EXPECT_EQ(accel_.getName(), "ADXL345");
}

TEST_F(AccelerometerTest, GetDeviceId) {
    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_DEVID, _))
        .WillOnce(DoAll(SetArgReferee<1>(Accelerometer::CHIP_ID), Return(Status::Ok)));
    EXPECT_EQ(accel_.getDeviceId(), 0xE5);
}

TEST_F(AccelerometerTest, IsConnected_True) {
    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_DEVID, _))
        .WillOnce(DoAll(SetArgReferee<1>(Accelerometer::CHIP_ID), Return(Status::Ok)));
    EXPECT_TRUE(accel_.isConnected());
}

TEST_F(AccelerometerTest, IsConnected_False_WrongId) {
    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_DEVID, _))
        .WillOnce(DoAll(SetArgReferee<1>(uint8_t{0xAA}), Return(Status::Ok)));
    EXPECT_FALSE(accel_.isConnected());
}

TEST_F(AccelerometerTest, IsConnected_False_SpiError) {
    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_DEVID, _))
        .WillOnce(Return(Status::Error));
    EXPECT_FALSE(accel_.isConnected());
}

TEST_F(AccelerometerTest, GetRange_DefaultIsG2) {
    EXPECT_EQ(accel_.getRange(), Accelerometer::Range::G2);
}

// ─── Not-initialized guards ───────────────────────────────────────────────────

TEST_F(AccelerometerTest, ReadRawAcceleration_NotInitialized) {
    int16_t x = 0, y = 0, z = 0;
    EXPECT_EQ(accel_.readRawAcceleration(x, y, z), Status::NotInitialized);
}

TEST_F(AccelerometerTest, ReadAcceleration_NotInitialized) {
    float x = 0, y = 0, z = 0;
    EXPECT_EQ(accel_.readAcceleration(x, y, z), Status::NotInitialized);
}

TEST_F(AccelerometerTest, SetRange_NotInitialized) {
    EXPECT_EQ(accel_.setRange(Accelerometer::Range::G4), Status::NotInitialized);
}

TEST_F(AccelerometerTest, SetDataRate_NotInitialized) {
    EXPECT_EQ(accel_.setDataRate(Accelerometer::DataRate::Hz200), Status::NotInitialized);
}

TEST_F(AccelerometerTest, SetMeasureMode_NotInitialized) {
    EXPECT_EQ(accel_.setMeasureMode(true), Status::NotInitialized);
}

// ─── readRawAcceleration ──────────────────────────────────────────────────────

TEST_F(AccelerometerTest, ReadRawAcceleration_Success) {
    initSuccessfully();

    // X: 0x00,0x01 → 256  Y: 0x00,0xFF → -256  Z: 0xE8,0x00 → 232
    static const uint8_t data[6] = {0x00, 0x01, 0x00, 0xFF, 0xE8, 0x00};
    EXPECT_CALL(spi_, readRegisters(Accelerometer::REG_DATAX0, _, Accelerometer::ACCEL_DATA_SIZE))
        .WillOnce(DoAll(SetArrayArgument<1>(data, data + 6), Return(Status::Ok)));

    int16_t x = 0, y = 0, z = 0;
    EXPECT_EQ(accel_.readRawAcceleration(x, y, z), Status::Ok);
    EXPECT_EQ(x, 256);
    EXPECT_EQ(y, -256);
    EXPECT_EQ(z, 232);
}

TEST_F(AccelerometerTest, ReadRawAcceleration_SpiError) {
    initSuccessfully();

    EXPECT_CALL(spi_, readRegisters(Accelerometer::REG_DATAX0, _, Accelerometer::ACCEL_DATA_SIZE))
        .WillOnce(Return(Status::Error));

    int16_t x = 0, y = 0, z = 0;
    EXPECT_EQ(accel_.readRawAcceleration(x, y, z), Status::Error);
}

TEST_F(AccelerometerTest, ReadRawAcceleration_ZeroValues) {
    initSuccessfully();

    static const uint8_t data[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    EXPECT_CALL(spi_, readRegisters(Accelerometer::REG_DATAX0, _, Accelerometer::ACCEL_DATA_SIZE))
        .WillOnce(DoAll(SetArrayArgument<1>(data, data + 6), Return(Status::Ok)));

    int16_t x = 0, y = 0, z = 0;
    EXPECT_EQ(accel_.readRawAcceleration(x, y, z), Status::Ok);
    EXPECT_EQ(x, 0);
    EXPECT_EQ(y, 0);
    EXPECT_EQ(z, 0);
}

TEST_F(AccelerometerTest, ReadRawAcceleration_NegativeValues) {
    initSuccessfully();

    // All negative: X=-1 (0xFF,0xFF), Y=-512 (0x00,0xFE), Z=-1 (0xFF,0xFF)
    static const uint8_t data[6] = {0xFF, 0xFF, 0x00, 0xFE, 0xFF, 0xFF};
    EXPECT_CALL(spi_, readRegisters(Accelerometer::REG_DATAX0, _, Accelerometer::ACCEL_DATA_SIZE))
        .WillOnce(DoAll(SetArrayArgument<1>(data, data + 6), Return(Status::Ok)));

    int16_t x = 0, y = 0, z = 0;
    EXPECT_EQ(accel_.readRawAcceleration(x, y, z), Status::Ok);
    EXPECT_EQ(x, -1);
    EXPECT_EQ(y, -512);
    EXPECT_EQ(z, -1);
}

// ─── readAcceleration ─────────────────────────────────────────────────────────

TEST_F(AccelerometerTest, ReadAcceleration_Success_FullResScale) {
    initSuccessfully();

    // X: 0x00,0x01 → raw=256 → 256*0.004=1.024g
    // Y: 0x00,0xFF → raw=-256 → -256*0.004=-1.024g
    // Z: 0xE8,0x00 → raw=232 → 232*0.004=0.928g
    static const uint8_t data[6] = {0x00, 0x01, 0x00, 0xFF, 0xE8, 0x00};
    EXPECT_CALL(spi_, readRegisters(Accelerometer::REG_DATAX0, _, Accelerometer::ACCEL_DATA_SIZE))
        .WillOnce(DoAll(SetArrayArgument<1>(data, data + 6), Return(Status::Ok)));

    float x = 0, y = 0, z = 0;
    EXPECT_EQ(accel_.readAcceleration(x, y, z), Status::Ok);
    EXPECT_FLOAT_EQ(x, 256.0f * 0.004f);   // 1.024g
    EXPECT_FLOAT_EQ(y, -256.0f * 0.004f);  // -1.024g
    EXPECT_FLOAT_EQ(z, 232.0f * 0.004f);   // 0.928g
}

TEST_F(AccelerometerTest, ReadAcceleration_ZeroData) {
    initSuccessfully();

    static const uint8_t data[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    EXPECT_CALL(spi_, readRegisters(Accelerometer::REG_DATAX0, _, Accelerometer::ACCEL_DATA_SIZE))
        .WillOnce(DoAll(SetArrayArgument<1>(data, data + 6), Return(Status::Ok)));

    float x = 0, y = 0, z = 0;
    EXPECT_EQ(accel_.readAcceleration(x, y, z), Status::Ok);
    EXPECT_FLOAT_EQ(x, 0.0f);
    EXPECT_FLOAT_EQ(y, 0.0f);
    EXPECT_FLOAT_EQ(z, 0.0f);
}

TEST_F(AccelerometerTest, ReadAcceleration_SpiError) {
    initSuccessfully();

    EXPECT_CALL(spi_, readRegisters(Accelerometer::REG_DATAX0, _, Accelerometer::ACCEL_DATA_SIZE))
        .WillOnce(Return(Status::Error));

    float x = 0, y = 0, z = 0;
    EXPECT_EQ(accel_.readAcceleration(x, y, z), Status::Error);
}

// ─── setRange ─────────────────────────────────────────────────────────────────

TEST_F(AccelerometerTest, SetRange_Success_G4) {
    initSuccessfully();

    // Read current DATA_FORMAT (0x08 = FULL_RES | G2)
    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_DATA_FORMAT, _))
        .WillOnce(DoAll(SetArgReferee<1>(uint8_t{0x08}), Return(Status::Ok)));
    // Write with new range: (0x08 & ~0x03) | 0x01 = 0x09
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_DATA_FORMAT, uint8_t{0x09}))
        .WillOnce(Return(Status::Ok));

    EXPECT_EQ(accel_.setRange(Accelerometer::Range::G4), Status::Ok);
    EXPECT_EQ(accel_.getRange(), Accelerometer::Range::G4);
}

TEST_F(AccelerometerTest, SetRange_Success_G16) {
    initSuccessfully();

    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_DATA_FORMAT, _))
        .WillOnce(DoAll(SetArgReferee<1>(uint8_t{0x08}), Return(Status::Ok)));
    // (0x08 & ~0x03) | 0x03 = 0x0B
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_DATA_FORMAT, uint8_t{0x0B}))
        .WillOnce(Return(Status::Ok));

    EXPECT_EQ(accel_.setRange(Accelerometer::Range::G16), Status::Ok);
    EXPECT_EQ(accel_.getRange(), Accelerometer::Range::G16);
}

TEST_F(AccelerometerTest, SetRange_SpiReadError) {
    initSuccessfully();

    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_DATA_FORMAT, _))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(accel_.setRange(Accelerometer::Range::G8), Status::Error);
}

TEST_F(AccelerometerTest, SetRange_SpiWriteError) {
    initSuccessfully();

    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_DATA_FORMAT, _))
        .WillOnce(DoAll(SetArgReferee<1>(uint8_t{0x08}), Return(Status::Ok)));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_DATA_FORMAT, _))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(accel_.setRange(Accelerometer::Range::G8), Status::Error);
}

// ─── setDataRate ──────────────────────────────────────────────────────────────

TEST_F(AccelerometerTest, SetDataRate_Success_Hz200) {
    initSuccessfully();

    // Read current BW_RATE (0x0A = 100 Hz)
    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_BW_RATE, _))
        .WillOnce(DoAll(SetArgReferee<1>(uint8_t{0x0A}), Return(Status::Ok)));
    // Write with new rate: (0x0A & ~0x0F) | 0x0B = 0x0B
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_BW_RATE, uint8_t{0x0B}))
        .WillOnce(Return(Status::Ok));

    EXPECT_EQ(accel_.setDataRate(Accelerometer::DataRate::Hz200), Status::Ok);
}

TEST_F(AccelerometerTest, SetDataRate_PreservesLowPowerBit) {
    initSuccessfully();

    // Current BW_RATE has LOW_POWER set: 0x1A = LOW_POWER | 100Hz
    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_BW_RATE, _))
        .WillOnce(DoAll(SetArgReferee<1>(uint8_t{0x1A}), Return(Status::Ok)));
    // (0x1A & ~0x0F) | 0x0D = 0x10 | 0x0D = 0x1D
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_BW_RATE, uint8_t{0x1D}))
        .WillOnce(Return(Status::Ok));

    EXPECT_EQ(accel_.setDataRate(Accelerometer::DataRate::Hz800), Status::Ok);
}

TEST_F(AccelerometerTest, SetDataRate_SpiReadError) {
    initSuccessfully();

    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_BW_RATE, _))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(accel_.setDataRate(Accelerometer::DataRate::Hz400), Status::Error);
}

// ─── setMeasureMode ───────────────────────────────────────────────────────────

TEST_F(AccelerometerTest, SetMeasureMode_Enable) {
    initSuccessfully();

    // Current POWER_CTL = 0x00 (standby)
    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_POWER_CTL, _))
        .WillOnce(DoAll(SetArgReferee<1>(uint8_t{0x00}), Return(Status::Ok)));
    // Set bit 3: 0x00 | 0x08 = 0x08
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_POWER_CTL, uint8_t{0x08}))
        .WillOnce(Return(Status::Ok));

    EXPECT_EQ(accel_.setMeasureMode(true), Status::Ok);
}

TEST_F(AccelerometerTest, SetMeasureMode_Disable) {
    initSuccessfully();

    // Current POWER_CTL = 0x08 (measure mode)
    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_POWER_CTL, _))
        .WillOnce(DoAll(SetArgReferee<1>(uint8_t{0x08}), Return(Status::Ok)));
    // Clear bit 3: 0x08 & ~0x08 = 0x00
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_POWER_CTL, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));

    EXPECT_EQ(accel_.setMeasureMode(false), Status::Ok);
}

TEST_F(AccelerometerTest, SetMeasureMode_PreservesOtherBits) {
    initSuccessfully();

    // Current POWER_CTL = 0x0C (measure + sleep bits)
    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_POWER_CTL, _))
        .WillOnce(DoAll(SetArgReferee<1>(uint8_t{0x0C}), Return(Status::Ok)));
    // Disable: 0x0C & ~0x08 = 0x04 (sleep bit preserved)
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_POWER_CTL, uint8_t{0x04}))
        .WillOnce(Return(Status::Ok));

    EXPECT_EQ(accel_.setMeasureMode(false), Status::Ok);
}

TEST_F(AccelerometerTest, SetMeasureMode_SpiReadError) {
    initSuccessfully();

    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_POWER_CTL, _))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(accel_.setMeasureMode(true), Status::Error);
}

// ─── Reset ────────────────────────────────────────────────────────────────────

TEST_F(AccelerometerTest, Reset_Success) {
    initSuccessfully();

    // Reset writes 8 registers in sequence
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_POWER_CTL, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_INT_ENABLE, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_DATA_FORMAT, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_BW_RATE, uint8_t{0x0A}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_FIFO_CTL, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_OFSX, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_OFSY, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_OFSZ, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));

    EXPECT_EQ(accel_.reset(), Status::Ok);
}

TEST_F(AccelerometerTest, Reset_ClearsInitialized) {
    initSuccessfully();

    // Expect all reset writes
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_POWER_CTL, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_INT_ENABLE, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_DATA_FORMAT, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_BW_RATE, uint8_t{0x0A}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_FIFO_CTL, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_OFSX, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_OFSY, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_OFSZ, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));

    EXPECT_EQ(accel_.reset(), Status::Ok);

    // After reset, reads should fail with NotInitialized
    int16_t x = 0, y = 0, z = 0;
    EXPECT_EQ(accel_.readRawAcceleration(x, y, z), Status::NotInitialized);
}

TEST_F(AccelerometerTest, Reset_SpiErrorOnFirstWrite) {
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_POWER_CTL, uint8_t{0x00}))
        .WillOnce(Return(Status::Error));

    EXPECT_EQ(accel_.reset(), Status::Error);
}

TEST_F(AccelerometerTest, Reset_ResetsRangeToG2) {
    initSuccessfully();

    // Change range to G8 first
    EXPECT_CALL(spi_, readRegister(Accelerometer::REG_DATA_FORMAT, _))
        .WillOnce(DoAll(SetArgReferee<1>(uint8_t{0x08}), Return(Status::Ok)));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_DATA_FORMAT, uint8_t{0x0A}))
        .WillOnce(Return(Status::Ok));
    ASSERT_EQ(accel_.setRange(Accelerometer::Range::G8), Status::Ok);
    EXPECT_EQ(accel_.getRange(), Accelerometer::Range::G8);

    // Now reset
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_POWER_CTL, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_INT_ENABLE, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_DATA_FORMAT, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_BW_RATE, uint8_t{0x0A}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_FIFO_CTL, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_OFSX, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_OFSY, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(spi_, writeRegister(Accelerometer::REG_OFSZ, uint8_t{0x00}))
        .WillOnce(Return(Status::Ok));

    EXPECT_EQ(accel_.reset(), Status::Ok);
    EXPECT_EQ(accel_.getRange(), Accelerometer::Range::G2);
}

// ─── Data format: LSB-first assembly and 2's complement ──────────────────────

TEST_F(AccelerometerTest, RawData_MaxPositive) {
    initSuccessfully();

    // 0x7FFF = 32767
    static const uint8_t data[6] = {0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F};
    EXPECT_CALL(spi_, readRegisters(Accelerometer::REG_DATAX0, _, Accelerometer::ACCEL_DATA_SIZE))
        .WillOnce(DoAll(SetArrayArgument<1>(data, data + 6), Return(Status::Ok)));

    int16_t x = 0, y = 0, z = 0;
    EXPECT_EQ(accel_.readRawAcceleration(x, y, z), Status::Ok);
    EXPECT_EQ(x, 32767);
    EXPECT_EQ(y, 32767);
    EXPECT_EQ(z, 32767);
}

TEST_F(AccelerometerTest, RawData_MaxNegative) {
    initSuccessfully();

    // 0x8000 = -32768
    static const uint8_t data[6] = {0x00, 0x80, 0x00, 0x80, 0x00, 0x80};
    EXPECT_CALL(spi_, readRegisters(Accelerometer::REG_DATAX0, _, Accelerometer::ACCEL_DATA_SIZE))
        .WillOnce(DoAll(SetArrayArgument<1>(data, data + 6), Return(Status::Ok)));

    int16_t x = 0, y = 0, z = 0;
    EXPECT_EQ(accel_.readRawAcceleration(x, y, z), Status::Ok);
    EXPECT_EQ(x, -32768);
    EXPECT_EQ(y, -32768);
    EXPECT_EQ(z, -32768);
}

TEST_F(AccelerometerTest, RawData_TwosComplement_MinusOne) {
    initSuccessfully();

    // 0xFFFF = -1 in int16_t
    static const uint8_t data[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    EXPECT_CALL(spi_, readRegisters(Accelerometer::REG_DATAX0, _, Accelerometer::ACCEL_DATA_SIZE))
        .WillOnce(DoAll(SetArrayArgument<1>(data, data + 6), Return(Status::Ok)));

    int16_t x = 0, y = 0, z = 0;
    EXPECT_EQ(accel_.readRawAcceleration(x, y, z), Status::Ok);
    EXPECT_EQ(x, -1);
    EXPECT_EQ(y, -1);
    EXPECT_EQ(z, -1);
}
