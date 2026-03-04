#include "drivers/temperature_sensor.hpp"

namespace sensor::drivers {

TemperatureSensor::TemperatureSensor(II2C& i2c, uint8_t address)
    : i2c_(i2c), address_(address) {}

Status TemperatureSensor::init() {
    if (!isConnected()) {
        return Status::DeviceNotFound;
    }

    Status s = readCalibration();
    if (s != Status::Ok) {
        return s;
    }

    // Configure: 1x oversampling for all, normal mode
    // ctrl_hum must be written before ctrl_meas to take effect
    s = i2c_.writeRegister(address_, REG_CTRL_HUM, 0x01);
    if (s != Status::Ok) return s;

    // osrs_t=1x(001), osrs_p=1x(001), mode=normal(11) → 0b001_001_11 = 0x27
    s = i2c_.writeRegister(address_, REG_CTRL_MEAS, 0x27);
    if (s != Status::Ok) return s;

    // config: t_sb=0.5ms(000), filter=off(000), spi3w_en=0 → 0x00
    s = i2c_.writeRegister(address_, REG_CONFIG, 0x00);
    if (s != Status::Ok) return s;

    initialized_ = true;
    return Status::Ok;
}

Status TemperatureSensor::reset() {
    Status s = i2c_.writeRegister(address_, REG_RESET, RESET_VALUE);
    if (s != Status::Ok) return s;
    initialized_ = false;
    return Status::Ok;
}

bool TemperatureSensor::isConnected() {
    uint8_t id = 0;
    if (i2c_.readRegister(address_, REG_CHIP_ID, id) != Status::Ok) {
        return false;
    }
    return id == CHIP_ID;
}

uint8_t TemperatureSensor::getDeviceId() {
    uint8_t id = 0;
    i2c_.readRegister(address_, REG_CHIP_ID, id);
    return id;
}

std::string TemperatureSensor::getName() const {
    return "BME280";
}

Status TemperatureSensor::readTemperature(float& tempC) {
    if (!initialized_) return Status::NotInitialized;

    int32_t rawTemp = 0, rawPress = 0, rawHum = 0;
    Status s = readRawData(rawTemp, rawPress, rawHum);
    if (s != Status::Ok) return s;

    int32_t tComp = compensateTemperature(rawTemp);
    tempC = static_cast<float>(tComp) / 100.0f;
    return Status::Ok;
}

Status TemperatureSensor::readAll(float& tempC, float& pressHPa, float& humPercent) {
    if (!initialized_) return Status::NotInitialized;

    int32_t rawTemp = 0, rawPress = 0, rawHum = 0;
    Status s = readRawData(rawTemp, rawPress, rawHum);
    if (s != Status::Ok) return s;

    // Temperature must be compensated first (sets tFine_)
    int32_t tComp = compensateTemperature(rawTemp);
    tempC = static_cast<float>(tComp) / 100.0f;

    // Pressure: result is Q24.8 Pa, convert to hPa
    uint32_t pComp = compensatePressure(rawPress);
    pressHPa = static_cast<float>(pComp) / 256.0f / 100.0f;

    // Humidity: result is Q22.10 %RH
    uint32_t hComp = compensateHumidity(rawHum);
    humPercent = static_cast<float>(hComp) / 1024.0f;

    return Status::Ok;
}

const TemperatureSensor::CalibrationData& TemperatureSensor::calibration() const {
    return cal_;
}

Status TemperatureSensor::setMode(uint8_t mode) {
    if (!initialized_) return Status::NotInitialized;
    if (mode > 0x03) return Status::InvalidParam;

    uint8_t ctrl = 0;
    Status s = i2c_.readRegister(address_, REG_CTRL_MEAS, ctrl);
    if (s != Status::Ok) return s;

    ctrl = static_cast<uint8_t>((ctrl & 0xFC) | (mode & 0x03));
    return i2c_.writeRegister(address_, REG_CTRL_MEAS, ctrl);
}

// --- Private helpers ---

Status TemperatureSensor::readCalibration() {
    // Read temperature and pressure calibration (0x88..0xA1, 26 bytes)
    uint8_t tpBuf[CALIB_TP_LENGTH] = {};
    Status s = i2c_.readRegisters(address_, REG_CALIB_TEMP_PRESS, tpBuf, CALIB_TP_LENGTH);
    if (s != Status::Ok) return s;

    cal_.dig_T1 = static_cast<uint16_t>(tpBuf[0]  | (static_cast<uint16_t>(tpBuf[1])  << 8U));
    cal_.dig_T2 = static_cast<int16_t>(static_cast<uint16_t>(tpBuf[2]  | (static_cast<uint16_t>(tpBuf[3])  << 8U)));
    cal_.dig_T3 = static_cast<int16_t>(static_cast<uint16_t>(tpBuf[4]  | (static_cast<uint16_t>(tpBuf[5])  << 8U)));

    cal_.dig_P1 = static_cast<uint16_t>(tpBuf[6]  | (static_cast<uint16_t>(tpBuf[7])  << 8U));
    cal_.dig_P2 = static_cast<int16_t>(static_cast<uint16_t>(tpBuf[8]  | (static_cast<uint16_t>(tpBuf[9])  << 8U)));
    cal_.dig_P3 = static_cast<int16_t>(static_cast<uint16_t>(tpBuf[10] | (static_cast<uint16_t>(tpBuf[11]) << 8U)));
    cal_.dig_P4 = static_cast<int16_t>(static_cast<uint16_t>(tpBuf[12] | (static_cast<uint16_t>(tpBuf[13]) << 8U)));
    cal_.dig_P5 = static_cast<int16_t>(static_cast<uint16_t>(tpBuf[14] | (static_cast<uint16_t>(tpBuf[15]) << 8U)));
    cal_.dig_P6 = static_cast<int16_t>(static_cast<uint16_t>(tpBuf[16] | (static_cast<uint16_t>(tpBuf[17]) << 8U)));
    cal_.dig_P7 = static_cast<int16_t>(static_cast<uint16_t>(tpBuf[18] | (static_cast<uint16_t>(tpBuf[19]) << 8U)));
    cal_.dig_P8 = static_cast<int16_t>(static_cast<uint16_t>(tpBuf[20] | (static_cast<uint16_t>(tpBuf[21]) << 8U)));
    cal_.dig_P9 = static_cast<int16_t>(static_cast<uint16_t>(tpBuf[22] | (static_cast<uint16_t>(tpBuf[23]) << 8U)));

    // Read humidity calibration H1 (0xA1, 1 byte)
    s = i2c_.readRegister(address_, REG_CALIB_HUM_H1, cal_.dig_H1);
    if (s != Status::Ok) return s;

    // Read humidity calibration H2..H6 (0xE1..0xE7, 7 bytes)
    uint8_t hBuf[CALIB_HUM_LENGTH] = {};
    s = i2c_.readRegisters(address_, REG_CALIB_HUM_H2, hBuf, CALIB_HUM_LENGTH);
    if (s != Status::Ok) return s;

    cal_.dig_H2 = static_cast<int16_t>(static_cast<uint16_t>(hBuf[0] | (static_cast<uint16_t>(hBuf[1]) << 8U)));
    cal_.dig_H3 = hBuf[2];
    // dig_H4: (0xE4 << 4) | (0xE5 & 0x0F)
    cal_.dig_H4 = static_cast<int16_t>(static_cast<int16_t>(static_cast<int8_t>(hBuf[3])) * 16 |
                                        (hBuf[4] & 0x0F));
    // dig_H5: (0xE6 << 4) | (0xE5 >> 4)
    cal_.dig_H5 = static_cast<int16_t>(static_cast<int16_t>(static_cast<int8_t>(hBuf[5])) * 16 |
                                        (hBuf[4] >> 4));
    cal_.dig_H6 = static_cast<int8_t>(hBuf[6]);

    return Status::Ok;
}

Status TemperatureSensor::readRawData(int32_t& rawTemp, int32_t& rawPress, int32_t& rawHum) {
    uint8_t buf[DATA_LENGTH] = {};
    Status s = i2c_.readRegisters(address_, REG_DATA_START, buf, DATA_LENGTH);
    if (s != Status::Ok) return s;

    // press_raw: buf[0]=msb, buf[1]=lsb, buf[2]=xlsb
    rawPress = (static_cast<int32_t>(buf[0]) << 12) |
               (static_cast<int32_t>(buf[1]) << 4)  |
               (static_cast<int32_t>(buf[2]) >> 4);

    // temp_raw: buf[3]=msb, buf[4]=lsb, buf[5]=xlsb
    rawTemp = (static_cast<int32_t>(buf[3]) << 12) |
              (static_cast<int32_t>(buf[4]) << 4)  |
              (static_cast<int32_t>(buf[5]) >> 4);

    // hum_raw: buf[6]=msb, buf[7]=lsb
    rawHum = (static_cast<int32_t>(buf[6]) << 8) |
              static_cast<int32_t>(buf[7]);

    return Status::Ok;
}

int32_t TemperatureSensor::compensateTemperature(int32_t adcT) {
    int32_t var1 = ((((adcT >> 3) - (static_cast<int32_t>(cal_.dig_T1) << 1))) *
                    static_cast<int32_t>(cal_.dig_T2)) >> 11;

    int32_t var2 = (((((adcT >> 4) - static_cast<int32_t>(cal_.dig_T1)) *
                      ((adcT >> 4) - static_cast<int32_t>(cal_.dig_T1))) >> 12) *
                    static_cast<int32_t>(cal_.dig_T3)) >> 14;

    tFine_ = var1 + var2;
    int32_t T = (tFine_ * 5 + 128) >> 8;
    return T;
}

uint32_t TemperatureSensor::compensatePressure(int32_t adcP) {
    int64_t var1 = static_cast<int64_t>(tFine_) - 128000;
    int64_t var2 = var1 * var1 * static_cast<int64_t>(cal_.dig_P6);
    var2 = var2 + ((var1 * static_cast<int64_t>(cal_.dig_P5)) << 17);
    var2 = var2 + (static_cast<int64_t>(cal_.dig_P4) << 35);
    var1 = ((var1 * var1 * static_cast<int64_t>(cal_.dig_P3)) >> 8) +
           ((var1 * static_cast<int64_t>(cal_.dig_P2)) << 12);
    var1 = ((static_cast<int64_t>(1) << 47) + var1) * static_cast<int64_t>(cal_.dig_P1) >> 33;

    if (var1 == 0) return 0;

    int64_t p = 1048576 - static_cast<int64_t>(adcP);
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (static_cast<int64_t>(cal_.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (static_cast<int64_t>(cal_.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (static_cast<int64_t>(cal_.dig_P7) << 4);

    return static_cast<uint32_t>(p);
}

uint32_t TemperatureSensor::compensateHumidity(int32_t adcH) {
    int32_t v = tFine_ - 76800;

    v = (((((adcH << 14) - (static_cast<int32_t>(cal_.dig_H4) << 20) -
            (static_cast<int32_t>(cal_.dig_H5) * v)) +
           16384) >> 15) *
         (((((((v * static_cast<int32_t>(cal_.dig_H6)) >> 10) *
              (((v * static_cast<int32_t>(cal_.dig_H3)) >> 11) + 32768)) >> 10) +
            2097152) * static_cast<int32_t>(cal_.dig_H2) + 8192) >> 14));

    v = v - (((((v >> 15) * (v >> 15)) >> 7) * static_cast<int32_t>(cal_.dig_H1)) >> 4);
    v = (v < 0 ? 0 : v);
    v = (v > 419430400 ? 419430400 : v);

    return static_cast<uint32_t>(v >> 12);
}

} // namespace sensor::drivers
