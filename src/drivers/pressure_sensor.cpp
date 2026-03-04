#include "drivers/pressure_sensor.hpp"

namespace sensor::drivers {

PressureSensor::PressureSensor(II2C& i2c, uint8_t address)
    : i2c_(i2c), address_(address) {}

Status PressureSensor::init() {
    // Verify chip ID
    uint8_t id = 0;
    Status s = i2c_.readRegister(address_, REG_CHIP_ID, id);
    if (s != Status::Ok) return s;
    if (id != CHIP_ID) return Status::DeviceNotFound;

    // Soft reset
    s = i2c_.writeRegister(address_, REG_RESET, RESET_CMD);
    if (s != Status::Ok) return s;

    // Read calibration data
    s = readCalibration();
    if (s != Status::Ok) return s;

    // Configure: set config first (must be written in sleep mode)
    s = i2c_.writeRegister(address_, REG_CONFIG, DEFAULT_CONFIG);
    if (s != Status::Ok) return s;

    // Set ctrl_meas (osrs_t=×1, osrs_p=×1, normal mode)
    s = i2c_.writeRegister(address_, REG_CTRL_MEAS, DEFAULT_CTRL_MEAS);
    if (s != Status::Ok) return s;

    initialized_ = true;
    return Status::Ok;
}

Status PressureSensor::reset() {
    Status s = i2c_.writeRegister(address_, REG_RESET, RESET_CMD);
    if (s != Status::Ok) return s;
    initialized_ = false;
    return Status::Ok;
}

bool PressureSensor::isConnected() {
    uint8_t id = 0;
    Status s = i2c_.readRegister(address_, REG_CHIP_ID, id);
    return (s == Status::Ok) && (id == CHIP_ID);
}

uint8_t PressureSensor::getDeviceId() {
    uint8_t id = 0;
    i2c_.readRegister(address_, REG_CHIP_ID, id);
    return id;
}

std::string PressureSensor::getName() const {
    return "BMP280";
}

Status PressureSensor::readPressure(float& pressHPa) {
    if (!initialized_) return Status::NotInitialized;

    int32_t rawTemp = 0;
    int32_t rawPress = 0;
    Status s = readRawData(rawTemp, rawPress);
    if (s != Status::Ok) return s;

    // Temperature must be compensated first to set tFine_
    compensateTemperature(rawTemp);
    uint32_t pressQ24_8 = compensatePressure(rawPress);
    // Convert Q24.8 Pa to hPa: divide by 256 to get Pa, then by 100 for hPa
    pressHPa = static_cast<float>(pressQ24_8) / 25600.0f;
    return Status::Ok;
}

Status PressureSensor::readTemperature(float& tempC) {
    if (!initialized_) return Status::NotInitialized;

    int32_t rawTemp = 0;
    int32_t rawPress = 0;
    Status s = readRawData(rawTemp, rawPress);
    if (s != Status::Ok) return s;

    int32_t temp100 = compensateTemperature(rawTemp);
    // Convert from 0.01°C to °C
    tempC = static_cast<float>(temp100) / 100.0f;
    return Status::Ok;
}

Status PressureSensor::readAll(float& tempC, float& pressHPa) {
    if (!initialized_) return Status::NotInitialized;

    int32_t rawTemp = 0;
    int32_t rawPress = 0;
    Status s = readRawData(rawTemp, rawPress);
    if (s != Status::Ok) return s;

    int32_t temp100 = compensateTemperature(rawTemp);
    tempC = static_cast<float>(temp100) / 100.0f;

    uint32_t pressQ24_8 = compensatePressure(rawPress);
    pressHPa = static_cast<float>(pressQ24_8) / 25600.0f;
    return Status::Ok;
}

const PressureSensor::CalibrationData& PressureSensor::calibration() const {
    return calib_;
}

Status PressureSensor::setMode(uint8_t mode) {
    if (!initialized_) return Status::NotInitialized;
    if (mode > MODE_NORMAL) return Status::InvalidParam;

    uint8_t ctrlMeas = 0;
    Status s = i2c_.readRegister(address_, REG_CTRL_MEAS, ctrlMeas);
    if (s != Status::Ok) return s;

    ctrlMeas = static_cast<uint8_t>((ctrlMeas & 0xFC) | (mode & 0x03));
    return i2c_.writeRegister(address_, REG_CTRL_MEAS, ctrlMeas);
}

Status PressureSensor::readCalibration() {
    uint8_t buf[CALIB_SIZE] = {};
    Status s = i2c_.readRegisters(address_, REG_CALIB_START, buf, CALIB_SIZE);
    if (s != Status::Ok) return s;

    calib_.dig_T1 = static_cast<uint16_t>(buf[0] | (static_cast<uint16_t>(buf[1]) << 8U));
    calib_.dig_T2 = static_cast<int16_t>(static_cast<uint16_t>(buf[2] | (static_cast<uint16_t>(buf[3]) << 8U)));
    calib_.dig_T3 = static_cast<int16_t>(static_cast<uint16_t>(buf[4] | (static_cast<uint16_t>(buf[5]) << 8U)));
    calib_.dig_P1 = static_cast<uint16_t>(buf[6] | (static_cast<uint16_t>(buf[7]) << 8U));
    calib_.dig_P2 = static_cast<int16_t>(static_cast<uint16_t>(buf[8] | (static_cast<uint16_t>(buf[9]) << 8U)));
    calib_.dig_P3 = static_cast<int16_t>(static_cast<uint16_t>(buf[10] | (static_cast<uint16_t>(buf[11]) << 8U)));
    calib_.dig_P4 = static_cast<int16_t>(static_cast<uint16_t>(buf[12] | (static_cast<uint16_t>(buf[13]) << 8U)));
    calib_.dig_P5 = static_cast<int16_t>(static_cast<uint16_t>(buf[14] | (static_cast<uint16_t>(buf[15]) << 8U)));
    calib_.dig_P6 = static_cast<int16_t>(static_cast<uint16_t>(buf[16] | (static_cast<uint16_t>(buf[17]) << 8U)));
    calib_.dig_P7 = static_cast<int16_t>(static_cast<uint16_t>(buf[18] | (static_cast<uint16_t>(buf[19]) << 8U)));
    calib_.dig_P8 = static_cast<int16_t>(static_cast<uint16_t>(buf[20] | (static_cast<uint16_t>(buf[21]) << 8U)));
    calib_.dig_P9 = static_cast<int16_t>(static_cast<uint16_t>(buf[22] | (static_cast<uint16_t>(buf[23]) << 8U)));

    return Status::Ok;
}

Status PressureSensor::readRawData(int32_t& rawTemp, int32_t& rawPress) {
    uint8_t buf[DATA_SIZE] = {};
    Status s = i2c_.readRegisters(address_, REG_DATA_START, buf, DATA_SIZE);
    if (s != Status::Ok) return s;

    // press: buf[0]=msb, buf[1]=lsb, buf[2]=xlsb
    rawPress = (static_cast<int32_t>(buf[0]) << 12)
             | (static_cast<int32_t>(buf[1]) << 4)
             | (static_cast<int32_t>(buf[2]) >> 4);

    // temp: buf[3]=msb, buf[4]=lsb, buf[5]=xlsb
    rawTemp = (static_cast<int32_t>(buf[3]) << 12)
            | (static_cast<int32_t>(buf[4]) << 4)
            | (static_cast<int32_t>(buf[5]) >> 4);

    return Status::Ok;
}

int32_t PressureSensor::compensateTemperature(int32_t adcT) {
    int32_t var1 = ((((adcT >> 3) - (static_cast<int32_t>(calib_.dig_T1) << 1)))
                    * static_cast<int32_t>(calib_.dig_T2)) >> 11;
    int32_t var2 = (((((adcT >> 4) - static_cast<int32_t>(calib_.dig_T1))
                    * ((adcT >> 4) - static_cast<int32_t>(calib_.dig_T1))) >> 12)
                    * static_cast<int32_t>(calib_.dig_T3)) >> 14;
    tFine_ = var1 + var2;
    return (tFine_ * 5 + 128) >> 8;
}

uint32_t PressureSensor::compensatePressure(int32_t adcP) {
    int64_t var1 = static_cast<int64_t>(tFine_) - 128000;
    int64_t var2 = var1 * var1 * static_cast<int64_t>(calib_.dig_P6);
    var2 = var2 + ((var1 * static_cast<int64_t>(calib_.dig_P5)) << 17);
    var2 = var2 + ((static_cast<int64_t>(calib_.dig_P4)) << 35);
    var1 = ((var1 * var1 * static_cast<int64_t>(calib_.dig_P3)) >> 8)
         + ((var1 * static_cast<int64_t>(calib_.dig_P2)) << 12);
    var1 = ((((static_cast<int64_t>(1)) << 47) + var1))
         * static_cast<int64_t>(calib_.dig_P1) >> 33;
    if (var1 == 0) return 0;

    int64_t p = 1048576 - static_cast<int64_t>(adcP);
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (static_cast<int64_t>(calib_.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (static_cast<int64_t>(calib_.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((static_cast<int64_t>(calib_.dig_P7)) << 4);
    return static_cast<uint32_t>(p);
}

} // namespace sensor::drivers
