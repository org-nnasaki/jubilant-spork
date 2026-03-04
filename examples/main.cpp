// EmbeddedSensorLib — Example program demonstrating all three sensor drivers
// with stub I2C/SPI implementations that return realistic dummy data.

#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

#include "sensor_base.hpp"
#include "i2c_interface.hpp"
#include "spi_interface.hpp"
#include "drivers/temperature_sensor.hpp"
#include "drivers/accelerometer.hpp"
#include "drivers/pressure_sensor.hpp"

// ---------------------------------------------------------------------------
// Helper: print a Status value
// ---------------------------------------------------------------------------
static const char* statusToString(sensor::Status s) {
    switch (s) {
        case sensor::Status::Ok:              return "Ok";
        case sensor::Status::Error:           return "Error";
        case sensor::Status::Timeout:         return "Timeout";
        case sensor::Status::InvalidParam:    return "InvalidParam";
        case sensor::Status::NotInitialized:  return "NotInitialized";
        case sensor::Status::DeviceNotFound:  return "DeviceNotFound";
    }
    return "Unknown";
}

// ---------------------------------------------------------------------------
// ExampleI2C — stub II2C serving BME280 (addr 0x76) and BMP280 (addr 0x77)
// ---------------------------------------------------------------------------
class ExampleI2C : public sensor::II2C {
public:
    sensor::Status readRegister(uint8_t devAddr, uint8_t regAddr,
                                uint8_t& value) override {
        // Chip ID register
        if (regAddr == 0xD0) {
            if (devAddr == 0x76) { value = 0x60; return sensor::Status::Ok; } // BME280
            if (devAddr == 0x77) { value = 0x58; return sensor::Status::Ok; } // BMP280
        }
        // Humidity control readback for BME280
        if (devAddr == 0x76 && regAddr == 0xA1) {
            value = 0x4B; // dig_H1 = 75
            return sensor::Status::Ok;
        }
        value = 0x00;
        return sensor::Status::Ok;
    }

    sensor::Status writeRegister(uint8_t /*devAddr*/, uint8_t /*regAddr*/,
                                 uint8_t /*value*/) override {
        return sensor::Status::Ok;
    }

    sensor::Status readRegisters(uint8_t devAddr, uint8_t regAddr,
                                 uint8_t* buffer, uint8_t length) override {
        if (buffer == nullptr || length == 0) {
            return sensor::Status::InvalidParam;
        }
        std::memset(buffer, 0, length);

        // --- BME280 (0x76) calibration & data ---
        if (devAddr == 0x76) {
            if (regAddr == 0x88 && length == 26) {
                // Temp/pressure calibration (matches BME280 datasheet example)
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
                std::memcpy(buffer, tpCal, 26);
                return sensor::Status::Ok;
            }
            if (regAddr == 0xE1 && length == 7) {
                // Humidity calibration H2..H6
                static const uint8_t hCal[7] = {
                    0x72, 0x01,  // dig_H2 = 370
                    0x00,        // dig_H3 = 0
                    0x13,        // dig_H4 high
                    0x92,        // H4/H5 nibbles
                    0x03,        // dig_H5 high
                    0x1E,        // dig_H6 = 30
                };
                std::memcpy(buffer, hCal, 7);
                return sensor::Status::Ok;
            }
            if (regAddr == 0xF7 && length == 8) {
                // Measurement data: press[3] | temp[3] | hum[2]
                static const uint8_t data[8] = {
                    0x50, 0x00, 0x00,  // pressure raw
                    0x7E, 0xF5, 0x00,  // temperature raw
                    0x80, 0x00,        // humidity raw
                };
                std::memcpy(buffer, data, 8);
                return sensor::Status::Ok;
            }
        }

        // --- BMP280 (0x77) calibration & data ---
        if (devAddr == 0x77) {
            if (regAddr == 0x88 && length == 24) {
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
                std::memcpy(buffer, cal, 24);
                return sensor::Status::Ok;
            }
            if (regAddr == 0xF7 && length == 6) {
                static const uint8_t data[6] = {
                    0x65, 0x5A, 0xC0,  // pressure raw
                    0x7E, 0xF5, 0x00,  // temperature raw
                };
                std::memcpy(buffer, data, 6);
                return sensor::Status::Ok;
            }
        }

        return sensor::Status::Ok;
    }

    sensor::Status writeRegisters(uint8_t /*devAddr*/, uint8_t /*regAddr*/,
                                  const uint8_t* /*buffer*/,
                                  uint8_t /*length*/) override {
        return sensor::Status::Ok;
    }
};

// ---------------------------------------------------------------------------
// ExampleSPI — stub ISPI serving ADXL345 accelerometer
// ---------------------------------------------------------------------------
class ExampleSPI : public sensor::ISPI {
public:
    sensor::Status readRegister(uint8_t regAddr, uint8_t& value) override {
        if (regAddr == 0x00) { // DEVID
            value = 0xE5;
            return sensor::Status::Ok;
        }
        value = 0x00;
        return sensor::Status::Ok;
    }

    sensor::Status writeRegister(uint8_t /*regAddr*/,
                                 uint8_t /*value*/) override {
        return sensor::Status::Ok;
    }

    sensor::Status readRegisters(uint8_t regAddr, uint8_t* buffer,
                                 uint8_t length) override {
        if (buffer == nullptr || length == 0) {
            return sensor::Status::InvalidParam;
        }
        std::memset(buffer, 0, length);

        if (regAddr == 0x32 && length == 6) {
            // X=256 (1.024 g), Y=-256 (-1.024 g), Z=232 (0.928 g)
            static const uint8_t data[6] = {
                0x00, 0x01,  // X LSB, MSB
                0x00, 0xFF,  // Y LSB, MSB
                0xE8, 0x00,  // Z LSB, MSB
            };
            std::memcpy(buffer, data, 6);
        }
        return sensor::Status::Ok;
    }

    sensor::Status writeRegisters(uint8_t /*regAddr*/,
                                  const uint8_t* /*buffer*/,
                                  uint8_t /*length*/) override {
        return sensor::Status::Ok;
    }
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void printHeader(const std::string& title) {
    std::cout << "\n========================================\n"
              << "  " << title
              << "\n========================================\n";
}

static void printStatus(const std::string& label, sensor::Status s) {
    std::cout << "  " << std::left << std::setw(22) << label
              << statusToString(s) << "\n";
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main() {
    using namespace sensor;
    using namespace sensor::drivers;

    std::cout << std::fixed << std::setprecision(2);

    // Create bus stubs
    ExampleI2C i2c;
    ExampleSPI spi;

    // Create sensor instances
    TemperatureSensor bme280(i2c, 0x76);
    Accelerometer     adxl345(spi);
    PressureSensor    bmp280(i2c, 0x77);

    // ==== BME280 Temperature / Humidity / Pressure ========================
    printHeader("BME280 — Temperature, Humidity & Pressure");

    Status st = bme280.init();
    printStatus("init():", st);
    std::cout << "  Name:      " << bme280.getName() << "\n";
    std::cout << "  Device ID: 0x" << std::hex << std::uppercase
              << static_cast<int>(bme280.getDeviceId())
              << std::dec << std::nouppercase << "\n";
    std::cout << "  Connected: " << (bme280.isConnected() ? "yes" : "no")
              << "\n";

    float tempC = 0.0F;
    float pressHPa = 0.0F;
    float humPct = 0.0F;

    st = bme280.readTemperature(tempC);
    printStatus("readTemperature():", st);
    std::cout << "  Temperature: " << tempC << " °C\n";

    st = bme280.readAll(tempC, pressHPa, humPct);
    printStatus("readAll():", st);
    std::cout << "  Temperature: " << tempC << " °C\n"
              << "  Pressure:    " << pressHPa << " hPa\n"
              << "  Humidity:    " << humPct << " %\n";

    st = bme280.setMode(TemperatureSensor::MODE_FORCED);
    printStatus("setMode(FORCED):", st);

    st = bme280.setMode(TemperatureSensor::MODE_NORMAL);
    printStatus("setMode(NORMAL):", st);

    // ==== ADXL345 Accelerometer ===========================================
    printHeader("ADXL345 — 3-Axis Accelerometer");

    st = adxl345.init();
    printStatus("init():", st);
    std::cout << "  Name:      " << adxl345.getName() << "\n";
    std::cout << "  Device ID: 0x" << std::hex << std::uppercase
              << static_cast<int>(adxl345.getDeviceId())
              << std::dec << std::nouppercase << "\n";
    std::cout << "  Connected: " << (adxl345.isConnected() ? "yes" : "no")
              << "\n";

    float ax = 0.0F;
    float ay = 0.0F;
    float az = 0.0F;

    st = adxl345.readAcceleration(ax, ay, az);
    printStatus("readAcceleration():", st);
    std::cout << "  X: " << std::setw(8) << ax << " g\n"
              << "  Y: " << std::setw(8) << ay << " g\n"
              << "  Z: " << std::setw(8) << az << " g\n";

    st = adxl345.setRange(Accelerometer::Range::G4);
    printStatus("setRange(G4):", st);

    st = adxl345.setRange(Accelerometer::Range::G16);
    printStatus("setRange(G16):", st);

    st = adxl345.setDataRate(Accelerometer::DataRate::Hz100);
    printStatus("setDataRate(100 Hz):", st);

    st = adxl345.setMeasureMode(true);
    printStatus("setMeasureMode(on):", st);

    // ==== BMP280 Pressure =================================================
    printHeader("BMP280 — Pressure & Temperature");

    st = bmp280.init();
    printStatus("init():", st);
    std::cout << "  Name:      " << bmp280.getName() << "\n";
    std::cout << "  Device ID: 0x" << std::hex << std::uppercase
              << static_cast<int>(bmp280.getDeviceId())
              << std::dec << std::nouppercase << "\n";
    std::cout << "  Connected: " << (bmp280.isConnected() ? "yes" : "no")
              << "\n";

    float bmpTemp = 0.0F;
    float bmpPress = 0.0F;

    st = bmp280.readTemperature(bmpTemp);
    printStatus("readTemperature():", st);
    std::cout << "  Temperature: " << bmpTemp << " °C\n";

    st = bmp280.readPressure(bmpPress);
    printStatus("readPressure():", st);
    std::cout << "  Pressure:    " << bmpPress << " hPa\n";

    st = bmp280.readAll(bmpTemp, bmpPress);
    printStatus("readAll():", st);
    std::cout << "  Temperature: " << bmpTemp << " °C\n"
              << "  Pressure:    " << bmpPress << " hPa\n";

    st = bmp280.setMode(PressureSensor::MODE_SLEEP);
    printStatus("setMode(SLEEP):", st);

    st = bmp280.setMode(PressureSensor::MODE_NORMAL);
    printStatus("setMode(NORMAL):", st);

    // ==== Summary =========================================================
    printHeader("Done");
    std::cout << "  All three sensor drivers demonstrated successfully.\n\n";

    return 0;
}
