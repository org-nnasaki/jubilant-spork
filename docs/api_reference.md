# EmbeddedSensorLib API Reference

## Table of Contents

- [Namespace `sensor`](#namespace-sensor)
  - [enum class Status](#enum-class-status)
  - [class ISensor](#class-isensor)
  - [class II2C](#class-ii2c)
  - [class ISPI](#class-ispi)
- [Namespace `sensor::drivers`](#namespace-sensordrivers)
  - [class TemperatureSensor](#class-temperaturesensor)
  - [class Accelerometer](#class-accelerometer)
  - [class PressureSensor](#class-pressuresensor)

---

## Namespace `sensor`

### enum class Status

Defined in: `include/sensor_base.hpp`

Return type for all sensor operations. Always check the returned status; never assume `Ok`.

```cpp
enum class Status : uint8_t {
    Ok = 0,
    Error,
    Timeout,
    InvalidParam,
    NotInitialized,
    DeviceNotFound
};
```

| Value            | Description                                              |
|------------------|----------------------------------------------------------|
| `Ok`             | Operation completed successfully.                        |
| `Error`          | Generic bus or device error.                             |
| `Timeout`        | Communication timed out waiting for the device.          |
| `InvalidParam`   | An invalid parameter was passed to the function.         |
| `NotInitialized` | Operation attempted before `init()` was called.          |
| `DeviceNotFound` | The expected device was not detected on the bus.         |

---

### class ISensor

Defined in: `include/sensor_base.hpp`

Abstract base class for all sensor drivers. Provides a common lifecycle interface. Instances are **non-copyable** but **movable**.

#### Destructor

```cpp
virtual ~ISensor() = default;
```

#### Pure Virtual Methods

##### `init`

```cpp
virtual Status init() = 0;
```

Initializes the sensor. Verifies device identity and reads factory calibration data. Must be called before any other sensor operation.

- **Returns:** `Status::Ok` on success, or an error status.

##### `reset`

```cpp
virtual Status reset() = 0;
```

Performs a soft reset of the sensor, restoring default register values.

- **Returns:** `Status::Ok` on success, or an error status.

##### `isConnected`

```cpp
virtual bool isConnected() = 0;
```

Checks whether the sensor is reachable on the bus by reading and verifying the chip ID register.

- **Returns:** `true` if the device responds with the expected chip ID.

##### `getDeviceId`

```cpp
virtual uint8_t getDeviceId() = 0;
```

Reads the chip ID register from the device.

- **Returns:** The raw chip ID byte read from the device.

##### `getName`

```cpp
virtual std::string getName() const = 0;
```

Returns a human-readable name for the sensor (e.g., `"BME280"`, `"ADXL345"`, `"BMP280"`).

- **Returns:** `std::string` containing the sensor name.

---

### class II2C

Defined in: `include/i2c_interface.hpp`

Pure-virtual I2C bus interface. Platform code must provide a concrete implementation. Test code uses GoogleMock.

#### Methods

##### `readRegister`

```cpp
virtual Status readRegister(uint8_t devAddr, uint8_t regAddr, uint8_t& value) = 0;
```

Reads a single byte from a register.

| Parameter | Type        | Description                         |
|-----------|-------------|-------------------------------------|
| `devAddr` | `uint8_t`   | 7-bit I2C device address.           |
| `regAddr` | `uint8_t`   | Register address to read from.      |
| `value`   | `uint8_t&`  | Output: the byte read.              |

- **Returns:** `Status::Ok` on success.

##### `writeRegister`

```cpp
virtual Status writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value) = 0;
```

Writes a single byte to a register.

| Parameter | Type       | Description                         |
|-----------|------------|-------------------------------------|
| `devAddr` | `uint8_t`  | 7-bit I2C device address.           |
| `regAddr` | `uint8_t`  | Register address to write to.       |
| `value`   | `uint8_t`  | Byte value to write.                |

- **Returns:** `Status::Ok` on success.

##### `readRegisters`

```cpp
virtual Status readRegisters(uint8_t devAddr, uint8_t regAddr, uint8_t* buffer, uint8_t length) = 0;
```

Reads a contiguous block of registers (burst read).

| Parameter | Type        | Description                                  |
|-----------|-------------|----------------------------------------------|
| `devAddr` | `uint8_t`   | 7-bit I2C device address.                    |
| `regAddr` | `uint8_t`   | Starting register address.                   |
| `buffer`  | `uint8_t*`  | Pointer to output buffer. Caller must allocate at least `length` bytes. |
| `length`  | `uint8_t`   | Number of bytes to read.                     |

- **Returns:** `Status::Ok` on success.

##### `writeRegisters`

```cpp
virtual Status writeRegisters(uint8_t devAddr, uint8_t regAddr, const uint8_t* buffer, uint8_t length) = 0;
```

Writes a contiguous block of registers (burst write).

| Parameter | Type              | Description                                  |
|-----------|-------------------|----------------------------------------------|
| `devAddr` | `uint8_t`         | 7-bit I2C device address.                    |
| `regAddr` | `uint8_t`         | Starting register address.                   |
| `buffer`  | `const uint8_t*`  | Pointer to data buffer to write.             |
| `length`  | `uint8_t`         | Number of bytes to write.                    |

- **Returns:** `Status::Ok` on success.

---

### class ISPI

Defined in: `include/spi_interface.hpp`

Pure-virtual SPI bus interface. Platform code must provide a concrete implementation. Test code uses GoogleMock. Unlike `II2C`, SPI methods do not take a device address (chip-select is managed externally).

#### Methods

##### `readRegister`

```cpp
virtual Status readRegister(uint8_t regAddr, uint8_t& value) = 0;
```

Reads a single byte from a register.

| Parameter | Type       | Description                    |
|-----------|------------|--------------------------------|
| `regAddr` | `uint8_t`  | Register address to read from. |
| `value`   | `uint8_t&` | Output: the byte read.         |

- **Returns:** `Status::Ok` on success.

##### `writeRegister`

```cpp
virtual Status writeRegister(uint8_t regAddr, uint8_t value) = 0;
```

Writes a single byte to a register.

| Parameter | Type       | Description                    |
|-----------|------------|--------------------------------|
| `regAddr` | `uint8_t`  | Register address to write to.  |
| `value`   | `uint8_t`  | Byte value to write.           |

- **Returns:** `Status::Ok` on success.

##### `readRegisters`

```cpp
virtual Status readRegisters(uint8_t regAddr, uint8_t* buffer, uint8_t length) = 0;
```

Reads a contiguous block of registers (burst read).

| Parameter | Type        | Description                                  |
|-----------|-------------|----------------------------------------------|
| `regAddr` | `uint8_t`   | Starting register address.                   |
| `buffer`  | `uint8_t*`  | Pointer to output buffer. Caller must allocate at least `length` bytes. |
| `length`  | `uint8_t`   | Number of bytes to read.                     |

- **Returns:** `Status::Ok` on success.

##### `writeRegisters`

```cpp
virtual Status writeRegisters(uint8_t regAddr, const uint8_t* buffer, uint8_t length) = 0;
```

Writes a contiguous block of registers (burst write).

| Parameter | Type              | Description                      |
|-----------|-------------------|----------------------------------|
| `regAddr` | `uint8_t`         | Starting register address.       |
| `buffer`  | `const uint8_t*`  | Pointer to data buffer to write. |
| `length`  | `uint8_t`         | Number of bytes to write.        |

- **Returns:** `Status::Ok` on success.

---

## Namespace `sensor::drivers`

### class TemperatureSensor

Defined in: `include/drivers/temperature_sensor.hpp`

Driver for the **Bosch BME280** combined temperature, pressure, and humidity sensor. Communicates over I2C. Inherits from `ISensor`.

**Chip ID:** `0x60` &nbsp;|&nbsp; **Default I2C address:** `0x76` &nbsp;|&nbsp; **Bus:** I2C

#### Constants

| Constant              | Value  | Description                            |
|-----------------------|--------|----------------------------------------|
| `REG_CHIP_ID`         | `0xD0` | Chip identification register.          |
| `REG_RESET`           | `0xE0` | Soft-reset register.                   |
| `REG_CTRL_HUM`        | `0xF2` | Humidity oversampling control.         |
| `REG_STATUS`          | `0xF3` | Device status register.                |
| `REG_CTRL_MEAS`       | `0xF4` | Temperature/pressure control register. |
| `REG_CONFIG`          | `0xF5` | Rate, filter, and interface config.    |
| `REG_DATA_START`      | `0xF7` | Start of burst data readout.           |
| `REG_CALIB_TEMP_PRESS`| `0x88` | Start of T/P calibration data.         |
| `REG_CALIB_HUM_H1`    | `0xA1` | Humidity calibration byte H1.          |
| `REG_CALIB_HUM_H2`    | `0xE1` | Start of humidity calibration H2–H6.   |
| `CHIP_ID`             | `0x60` | Expected chip ID for BME280.           |
| `RESET_VALUE`         | `0xB6` | Soft-reset command byte.               |
| `DATA_LENGTH`         | `8`    | Bytes in a full data burst read.       |
| `CALIB_TP_LENGTH`     | `26`   | Bytes in T/P calibration block.        |
| `CALIB_HUM_LENGTH`    | `7`    | Bytes in humidity calibration block.   |
| `MODE_SLEEP`          | `0x00` | Sleep mode (no measurements).          |
| `MODE_FORCED`         | `0x01` | Single-measurement mode.               |
| `MODE_NORMAL`         | `0x03` | Continuous measurement mode.           |

#### CalibrationData Struct

Factory calibration coefficients read during `init()`. Accessible via `calibration()`.

```cpp
struct CalibrationData {
    uint16_t dig_T1;  int16_t dig_T2;  int16_t dig_T3;           // Temperature
    uint16_t dig_P1;  int16_t dig_P2;  int16_t dig_P3;           // Pressure
    int16_t  dig_P4;  int16_t dig_P5;  int16_t dig_P6;
    int16_t  dig_P7;  int16_t dig_P8;  int16_t dig_P9;
    uint8_t  dig_H1;  int16_t dig_H2;  uint8_t dig_H3;           // Humidity
    int16_t  dig_H4;  int16_t dig_H5;  int8_t  dig_H6;
};
```

#### Constructor

```cpp
explicit TemperatureSensor(II2C& i2c, uint8_t address = 0x76);
```

| Parameter | Type       | Description                                       |
|-----------|------------|---------------------------------------------------|
| `i2c`     | `II2C&`    | Reference to an I2C bus implementation. Caller owns the lifetime. |
| `address` | `uint8_t`  | 7-bit I2C address. Default `0x76`; alternate `0x77`. |

#### ISensor Overrides

##### `init`

```cpp
Status init() override;
```

Verifies chip ID (`0x60`), performs a soft reset, and reads calibration data. Must be called before reading sensor values.

- **Returns:** `Status::Ok` on success; `Status::DeviceNotFound` if chip ID does not match.

##### `reset`

```cpp
Status reset() override;
```

Writes `RESET_VALUE` (`0xB6`) to `REG_RESET`.

- **Returns:** `Status::Ok` on success.

##### `isConnected`

```cpp
bool isConnected() override;
```

- **Returns:** `true` if reading `REG_CHIP_ID` returns `0x60`.

##### `getDeviceId`

```cpp
uint8_t getDeviceId() override;
```

- **Returns:** The raw byte read from `REG_CHIP_ID`.

##### `getName`

```cpp
std::string getName() const override;
```

- **Returns:** `"BME280"`.

#### Sensor-Specific Methods

##### `readTemperature`

```cpp
Status readTemperature(float& tempC);
```

Reads and compensates the temperature value.

| Parameter | Type     | Description                                 |
|-----------|----------|---------------------------------------------|
| `tempC`   | `float&` | Output: temperature in degrees Celsius (°C). |

- **Returns:** `Status::Ok` on success; `Status::NotInitialized` if `init()` has not been called.

##### `readAll`

```cpp
Status readAll(float& tempC, float& pressHPa, float& humPercent);
```

Reads temperature, pressure, and humidity in a single burst.

| Parameter    | Type     | Description                               |
|--------------|----------|-------------------------------------------|
| `tempC`      | `float&` | Output: temperature in °C.                |
| `pressHPa`   | `float&` | Output: pressure in hectopascals (hPa).   |
| `humPercent`  | `float&` | Output: relative humidity in percent (%).  |

- **Returns:** `Status::Ok` on success.

##### `calibration`

```cpp
const CalibrationData& calibration() const;
```

Returns a const reference to the stored factory calibration coefficients. Valid only after a successful `init()`.

- **Returns:** `const CalibrationData&`.

##### `setMode`

```cpp
Status setMode(uint8_t mode);
```

Sets the power/measurement mode.

| Parameter | Type      | Description                                              |
|-----------|-----------|----------------------------------------------------------|
| `mode`    | `uint8_t` | One of `MODE_SLEEP` (`0x00`), `MODE_FORCED` (`0x01`), or `MODE_NORMAL` (`0x03`). |

- **Returns:** `Status::Ok` on success.

#### Usage Example

```cpp
MockI2C i2c;  // or a real platform I2C implementation
sensor::drivers::TemperatureSensor bme(i2c, 0x76);

if (bme.init() == sensor::Status::Ok) {
    float tempC, pressHPa, humPercent;
    if (bme.readAll(tempC, pressHPa, humPercent) == sensor::Status::Ok) {
        // Use tempC, pressHPa, humPercent
    }
}
```

---

### class Accelerometer

Defined in: `include/drivers/accelerometer.hpp`

Driver for the **Analog Devices ADXL345** 3-axis digital accelerometer. Communicates over SPI. Inherits from `ISensor`.

**Chip ID:** `0xE5` &nbsp;|&nbsp; **Bus:** SPI

#### Constants

| Constant                | Value  | Description                               |
|-------------------------|--------|-------------------------------------------|
| `REG_DEVID`             | `0x00` | Device ID register.                       |
| `REG_OFSX`              | `0x1E` | X-axis offset register.                   |
| `REG_OFSY`              | `0x1F` | Y-axis offset register.                   |
| `REG_OFSZ`              | `0x20` | Z-axis offset register.                   |
| `REG_BW_RATE`           | `0x2C` | Data rate and power mode control.         |
| `REG_POWER_CTL`         | `0x2D` | Power-saving features control.            |
| `REG_INT_ENABLE`        | `0x2E` | Interrupt enable control.                 |
| `REG_INT_MAP`           | `0x2F` | Interrupt mapping control.                |
| `REG_INT_SOURCE`        | `0x30` | Source of interrupts.                     |
| `REG_DATA_FORMAT`       | `0x31` | Data format control (range, resolution).  |
| `REG_DATAX0`            | `0x32` | Start of X-axis data (6 bytes X/Y/Z).    |
| `REG_FIFO_CTL`          | `0x38` | FIFO control register.                    |
| `REG_FIFO_STATUS`       | `0x39` | FIFO status register.                     |
| `CHIP_ID`               | `0xE5` | Expected device ID for ADXL345.           |
| `DATA_FORMAT_FULL_RES`  | `0x08` | Full-resolution bit in DATA_FORMAT.       |
| `DATA_FORMAT_RANGE_MASK`| `0x03` | Range bits mask in DATA_FORMAT.           |
| `POWER_CTL_MEASURE`     | `0x08` | Measure mode bit in POWER_CTL.            |
| `POWER_CTL_SLEEP`       | `0x04` | Sleep mode bit in POWER_CTL.              |
| `BW_RATE_LOW_POWER`     | `0x10` | Low-power bit in BW_RATE.                 |
| `BW_RATE_RATE_MASK`     | `0x0F` | Rate bits mask in BW_RATE.                |
| `FULL_RES_SCALE`        | `0.004f` | Full-resolution scale: 4 mg/LSB.       |
| `ACCEL_DATA_SIZE`       | `6`    | Bytes in acceleration data (X/Y/Z).       |

#### Enumerations

##### `Range`

```cpp
enum class Range : uint8_t {
    G2  = 0x00,   // ±2 g
    G4  = 0x01,   // ±4 g
    G8  = 0x02,   // ±8 g
    G16 = 0x03    // ±16 g
};
```

##### `DataRate`

```cpp
enum class DataRate : uint8_t {
    Hz0_10 = 0x00,   //   0.10 Hz
    Hz0_20 = 0x01,   //   0.20 Hz
    Hz0_39 = 0x02,   //   0.39 Hz
    Hz0_78 = 0x03,   //   0.78 Hz
    Hz1_56 = 0x04,   //   1.56 Hz
    Hz3_13 = 0x05,   //   3.13 Hz
    Hz6_25 = 0x06,   //   6.25 Hz
    Hz12_5 = 0x07,   //  12.5  Hz
    Hz25   = 0x08,   //  25    Hz
    Hz50   = 0x09,   //  50    Hz
    Hz100  = 0x0A,   // 100    Hz
    Hz200  = 0x0B,   // 200    Hz
    Hz400  = 0x0C,   // 400    Hz
    Hz800  = 0x0D,   // 800    Hz
    Hz1600 = 0x0E,   // 1600   Hz
    Hz3200 = 0x0F    // 3200   Hz
};
```

##### `FifoMode`

```cpp
enum class FifoMode : uint8_t {
    Bypass  = 0x00,
    Fifo    = 0x01,
    Stream  = 0x02,
    Trigger = 0x03
};
```

#### Constructor

```cpp
explicit Accelerometer(ISPI& spi);
```

| Parameter | Type     | Description                                                       |
|-----------|----------|-------------------------------------------------------------------|
| `spi`     | `ISPI&`  | Reference to an SPI bus implementation. Caller owns the lifetime. |

#### ISensor Overrides

##### `init`

```cpp
Status init() override;
```

Verifies chip ID (`0xE5`), configures full-resolution mode and ±2 g range, and enables measurement mode.

- **Returns:** `Status::Ok` on success; `Status::DeviceNotFound` if chip ID does not match.

##### `reset`

```cpp
Status reset() override;
```

Resets key registers (`POWER_CTL`, `DATA_FORMAT`, `BW_RATE`, `INT_ENABLE`, `FIFO_CTL`, offsets) to their default values.

- **Returns:** `Status::Ok` on success.

##### `isConnected`

```cpp
bool isConnected() override;
```

- **Returns:** `true` if reading `REG_DEVID` returns `0xE5`.

##### `getDeviceId`

```cpp
uint8_t getDeviceId() override;
```

- **Returns:** The raw byte read from `REG_DEVID`.

##### `getName`

```cpp
std::string getName() const override;
```

- **Returns:** `"ADXL345"`.

#### Sensor-Specific Methods

##### `readAcceleration`

```cpp
Status readAcceleration(float& x, float& y, float& z);
```

Reads acceleration on all three axes, scaled to g units.

| Parameter | Type     | Description                      |
|-----------|----------|----------------------------------|
| `x`       | `float&` | Output: X-axis acceleration (g). |
| `y`       | `float&` | Output: Y-axis acceleration (g). |
| `z`       | `float&` | Output: Z-axis acceleration (g). |

- **Returns:** `Status::Ok` on success; `Status::NotInitialized` if `init()` has not been called.

##### `readRawAcceleration`

```cpp
Status readRawAcceleration(int16_t& x, int16_t& y, int16_t& z);
```

Reads raw (unscaled) acceleration counts on all three axes.

| Parameter | Type       | Description                             |
|-----------|------------|-----------------------------------------|
| `x`       | `int16_t&` | Output: raw X-axis count.               |
| `y`       | `int16_t&` | Output: raw Y-axis count.               |
| `z`       | `int16_t&` | Output: raw Z-axis count.               |

- **Returns:** `Status::Ok` on success; `Status::NotInitialized` if `init()` has not been called.

##### `setRange`

```cpp
Status setRange(Range range);
```

Sets the measurement range.

| Parameter | Type    | Description                                           |
|-----------|---------|-------------------------------------------------------|
| `range`   | `Range` | One of `G2`, `G4`, `G8`, `G16`.                      |

- **Returns:** `Status::Ok` on success.

##### `setDataRate`

```cpp
Status setDataRate(DataRate rate);
```

Sets the output data rate.

| Parameter | Type       | Description                                      |
|-----------|------------|--------------------------------------------------|
| `rate`    | `DataRate` | One of the `DataRate` enum values (0.10–3200 Hz).|

- **Returns:** `Status::Ok` on success.

##### `setMeasureMode`

```cpp
Status setMeasureMode(bool enable);
```

Enables or disables measurement mode.

| Parameter | Type   | Description                                  |
|-----------|--------|----------------------------------------------|
| `enable`  | `bool` | `true` to start measuring, `false` to stop.  |

- **Returns:** `Status::Ok` on success.

##### `getRange`

```cpp
Range getRange() const;
```

Returns the currently configured measurement range.

- **Returns:** Current `Range` value.

#### Usage Example

```cpp
MockSPI spi;  // or a real platform SPI implementation
sensor::drivers::Accelerometer accel(spi);

if (accel.init() == sensor::Status::Ok) {
    accel.setRange(sensor::drivers::Accelerometer::Range::G4);
    float x, y, z;
    if (accel.readAcceleration(x, y, z) == sensor::Status::Ok) {
        // Use x, y, z (in g)
    }
}
```

---

### class PressureSensor

Defined in: `include/drivers/pressure_sensor.hpp`

Driver for the **Bosch BMP280** barometric pressure and temperature sensor. Communicates over I2C. Inherits from `ISensor`.

**Chip ID:** `0x58` &nbsp;|&nbsp; **Default I2C address:** `0x76` &nbsp;|&nbsp; **Bus:** I2C

#### Constants

| Constant           | Value  | Description                            |
|--------------------|--------|----------------------------------------|
| `REG_CHIP_ID`      | `0xD0` | Chip identification register.          |
| `REG_RESET`        | `0xE0` | Soft-reset register.                   |
| `REG_STATUS`       | `0xF3` | Device status register.                |
| `REG_CTRL_MEAS`    | `0xF4` | Measurement control register.          |
| `REG_CONFIG`       | `0xF5` | Rate, filter, and interface config.    |
| `REG_DATA_START`   | `0xF7` | Start of burst data readout.           |
| `REG_CALIB_START`  | `0x88` | Start of calibration data.             |
| `CHIP_ID`          | `0x58` | Expected chip ID for BMP280.           |
| `RESET_CMD`        | `0xB6` | Soft-reset command byte.               |
| `CALIB_SIZE`       | `24`   | Bytes in calibration data block.       |
| `DATA_SIZE`        | `6`    | Bytes in a full data burst read.       |
| `MODE_SLEEP`       | `0x00` | Sleep mode (no measurements).          |
| `MODE_FORCED`      | `0x01` | Single-measurement mode.               |
| `MODE_NORMAL`      | `0x03` | Continuous measurement mode.           |
| `DEFAULT_CTRL_MEAS`| `0x27` | Default: osrs_t=×1, osrs_p=×1, normal.|
| `DEFAULT_CONFIG`   | `0x00` | Default: t_sb=0.5 ms, filter off.      |

#### CalibrationData Struct

Factory calibration coefficients read during `init()`. Accessible via `calibration()`.

```cpp
struct CalibrationData {
    uint16_t dig_T1;  int16_t dig_T2;  int16_t dig_T3;       // Temperature
    uint16_t dig_P1;  int16_t dig_P2;  int16_t dig_P3;       // Pressure
    int16_t  dig_P4;  int16_t dig_P5;  int16_t dig_P6;
    int16_t  dig_P7;  int16_t dig_P8;  int16_t dig_P9;
};
```

#### Constructor

```cpp
explicit PressureSensor(II2C& i2c, uint8_t address = 0x76);
```

| Parameter | Type       | Description                                       |
|-----------|------------|---------------------------------------------------|
| `i2c`     | `II2C&`    | Reference to an I2C bus implementation. Caller owns the lifetime. |
| `address` | `uint8_t`  | 7-bit I2C address. Default `0x76`; alternate `0x77`. |

#### ISensor Overrides

##### `init`

```cpp
Status init() override;
```

Verifies chip ID (`0x58`), performs a soft reset, reads calibration data, and configures default measurement settings (`DEFAULT_CTRL_MEAS`, `DEFAULT_CONFIG`).

- **Returns:** `Status::Ok` on success; `Status::DeviceNotFound` if chip ID does not match.

##### `reset`

```cpp
Status reset() override;
```

Writes `RESET_CMD` (`0xB6`) to `REG_RESET`.

- **Returns:** `Status::Ok` on success.

##### `isConnected`

```cpp
bool isConnected() override;
```

- **Returns:** `true` if reading `REG_CHIP_ID` returns `0x58`.

##### `getDeviceId`

```cpp
uint8_t getDeviceId() override;
```

- **Returns:** The raw byte read from `REG_CHIP_ID`.

##### `getName`

```cpp
std::string getName() const override;
```

- **Returns:** `"BMP280"`.

#### Sensor-Specific Methods

##### `readPressure`

```cpp
Status readPressure(float& pressHPa);
```

Reads and compensates the barometric pressure.

| Parameter  | Type     | Description                                  |
|------------|----------|----------------------------------------------|
| `pressHPa` | `float&` | Output: pressure in hectopascals (hPa).      |

- **Returns:** `Status::Ok` on success; `Status::NotInitialized` if `init()` has not been called.
- **Note:** Internally reads temperature first to compute the `tFine` correction value required by the pressure compensation formula.

##### `readTemperature`

```cpp
Status readTemperature(float& tempC);
```

Reads and compensates the temperature value.

| Parameter | Type     | Description                                  |
|-----------|----------|----------------------------------------------|
| `tempC`   | `float&` | Output: temperature in degrees Celsius (°C). |

- **Returns:** `Status::Ok` on success; `Status::NotInitialized` if `init()` has not been called.

##### `readAll`

```cpp
Status readAll(float& tempC, float& pressHPa);
```

Reads temperature and pressure in a single burst.

| Parameter  | Type     | Description                             |
|------------|----------|-----------------------------------------|
| `tempC`    | `float&` | Output: temperature in °C.              |
| `pressHPa` | `float&` | Output: pressure in hectopascals (hPa). |

- **Returns:** `Status::Ok` on success.

##### `calibration`

```cpp
const CalibrationData& calibration() const;
```

Returns a const reference to the stored factory calibration coefficients. Valid only after a successful `init()`.

- **Returns:** `const CalibrationData&`.

##### `setMode`

```cpp
Status setMode(uint8_t mode);
```

Sets the power/measurement mode.

| Parameter | Type      | Description                                              |
|-----------|-----------|----------------------------------------------------------|
| `mode`    | `uint8_t` | One of `MODE_SLEEP` (`0x00`), `MODE_FORCED` (`0x01`), or `MODE_NORMAL` (`0x03`). |

- **Returns:** `Status::Ok` on success.

#### Usage Example

```cpp
MockI2C i2c;  // or a real platform I2C implementation
sensor::drivers::PressureSensor bmp(i2c, 0x76);

if (bmp.init() == sensor::Status::Ok) {
    float tempC, pressHPa;
    if (bmp.readAll(tempC, pressHPa) == sensor::Status::Ok) {
        // Use tempC (°C) and pressHPa (hPa)
    }
}
```
