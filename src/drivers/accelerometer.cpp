#include "drivers/accelerometer.hpp"

namespace sensor::drivers {

Accelerometer::Accelerometer(ISPI& spi)
    : spi_(spi) {}

Status Accelerometer::init() {
    // 1) Verify chip ID
    uint8_t id = 0;
    Status status = readRegister(REG_DEVID, id);
    if (status != Status::Ok) {
        return status;
    }
    if (id != CHIP_ID) {
        return Status::DeviceNotFound;
    }

    // 2) Set DATA_FORMAT: full resolution, ±2g
    uint8_t dataFormat = DATA_FORMAT_FULL_RES | static_cast<uint8_t>(Range::G2);
    status = writeRegister(REG_DATA_FORMAT, dataFormat);
    if (status != Status::Ok) {
        return status;
    }
    fullResolution_ = true;
    range_ = Range::G2;

    // 3) Set BW_RATE: 100 Hz (default)
    status = writeRegister(REG_BW_RATE, static_cast<uint8_t>(DataRate::Hz100));
    if (status != Status::Ok) {
        return status;
    }

    // 4) Enable measurement mode
    status = writeRegister(REG_POWER_CTL, POWER_CTL_MEASURE);
    if (status != Status::Ok) {
        return status;
    }

    initialized_ = true;
    return Status::Ok;
}

Status Accelerometer::reset() {
    // Put device into standby
    Status status = writeRegister(REG_POWER_CTL, 0x00);
    if (status != Status::Ok) {
        return status;
    }

    // Clear interrupt enable
    status = writeRegister(REG_INT_ENABLE, 0x00);
    if (status != Status::Ok) {
        return status;
    }

    // Reset DATA_FORMAT to default (±2g, 10-bit)
    status = writeRegister(REG_DATA_FORMAT, 0x00);
    if (status != Status::Ok) {
        return status;
    }

    // Reset BW_RATE to default (100 Hz)
    status = writeRegister(REG_BW_RATE, static_cast<uint8_t>(DataRate::Hz100));
    if (status != Status::Ok) {
        return status;
    }

    // Reset FIFO to bypass
    status = writeRegister(REG_FIFO_CTL, 0x00);
    if (status != Status::Ok) {
        return status;
    }

    // Clear offsets
    status = writeRegister(REG_OFSX, 0x00);
    if (status != Status::Ok) {
        return status;
    }
    status = writeRegister(REG_OFSY, 0x00);
    if (status != Status::Ok) {
        return status;
    }
    status = writeRegister(REG_OFSZ, 0x00);
    if (status != Status::Ok) {
        return status;
    }

    initialized_ = false;
    fullResolution_ = true;
    range_ = Range::G2;

    return Status::Ok;
}

bool Accelerometer::isConnected() {
    uint8_t id = 0;
    Status status = readRegister(REG_DEVID, id);
    return (status == Status::Ok) && (id == CHIP_ID);
}

uint8_t Accelerometer::getDeviceId() {
    uint8_t id = 0;
    readRegister(REG_DEVID, id);
    return id;
}

std::string Accelerometer::getName() const {
    return "ADXL345";
}

Status Accelerometer::readAcceleration(float& x, float& y, float& z) {
    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;

    Status status = readRawAcceleration(rawX, rawY, rawZ);
    if (status != Status::Ok) {
        return status;
    }

    float scale = fullResolution_ ? FULL_RES_SCALE : scaleFactorForRange(range_);
    x = static_cast<float>(rawX) * scale;
    y = static_cast<float>(rawY) * scale;
    z = static_cast<float>(rawZ) * scale;

    return Status::Ok;
}

Status Accelerometer::readRawAcceleration(int16_t& x, int16_t& y, int16_t& z) {
    if (!initialized_) {
        return Status::NotInitialized;
    }

    uint8_t buffer[ACCEL_DATA_SIZE] = {};
    Status status = spi_.readRegisters(REG_DATAX0, buffer, ACCEL_DATA_SIZE);
    if (status != Status::Ok) {
        return status;
    }

    // LSB first, signed 16-bit: (HIGH << 8) | LOW
    x = static_cast<int16_t>(
        static_cast<uint16_t>(buffer[0]) |
        (static_cast<uint16_t>(buffer[1]) << 8U));
    y = static_cast<int16_t>(
        static_cast<uint16_t>(buffer[2]) |
        (static_cast<uint16_t>(buffer[3]) << 8U));
    z = static_cast<int16_t>(
        static_cast<uint16_t>(buffer[4]) |
        (static_cast<uint16_t>(buffer[5]) << 8U));

    return Status::Ok;
}

Status Accelerometer::setRange(Range range) {
    if (!initialized_) {
        return Status::NotInitialized;
    }

    uint8_t current = 0;
    Status status = readRegister(REG_DATA_FORMAT, current);
    if (status != Status::Ok) {
        return status;
    }

    uint8_t value = static_cast<uint8_t>(
        (current & static_cast<uint8_t>(~DATA_FORMAT_RANGE_MASK)) |
        static_cast<uint8_t>(range));
    status = writeRegister(REG_DATA_FORMAT, value);
    if (status != Status::Ok) {
        return status;
    }

    range_ = range;
    fullResolution_ = (current & DATA_FORMAT_FULL_RES) != 0;
    return Status::Ok;
}

Status Accelerometer::setDataRate(DataRate rate) {
    if (!initialized_) {
        return Status::NotInitialized;
    }

    uint8_t current = 0;
    Status status = readRegister(REG_BW_RATE, current);
    if (status != Status::Ok) {
        return status;
    }

    uint8_t value = static_cast<uint8_t>(
        (current & static_cast<uint8_t>(~BW_RATE_RATE_MASK)) |
        static_cast<uint8_t>(rate));
    return writeRegister(REG_BW_RATE, value);
}

Status Accelerometer::setMeasureMode(bool enable) {
    if (!initialized_) {
        return Status::NotInitialized;
    }

    uint8_t current = 0;
    Status status = readRegister(REG_POWER_CTL, current);
    if (status != Status::Ok) {
        return status;
    }

    uint8_t value = enable
        ? static_cast<uint8_t>(current | POWER_CTL_MEASURE)
        : static_cast<uint8_t>(current & static_cast<uint8_t>(~POWER_CTL_MEASURE));
    return writeRegister(REG_POWER_CTL, value);
}

Accelerometer::Range Accelerometer::getRange() const {
    return range_;
}

Status Accelerometer::readRegister(uint8_t reg, uint8_t& value) {
    return spi_.readRegister(reg, value);
}

Status Accelerometer::writeRegister(uint8_t reg, uint8_t value) {
    return spi_.writeRegister(reg, value);
}

float Accelerometer::scaleFactorForRange(Range range) const {
    // 10-bit mode scale factors in g/LSB
    switch (range) {
        case Range::G2:  return 0.0039f;
        case Range::G4:  return 0.0078f;
        case Range::G8:  return 0.0156f;
        case Range::G16: return 0.0312f;
    }
    return 0.0039f;
}

} // namespace sensor::drivers
