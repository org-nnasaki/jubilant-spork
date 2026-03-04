# EmbeddedSensorLib

A C++17 embedded sensor driver library with hardware abstraction via pure-virtual bus interfaces. Platform-independent drivers communicate through `II2C` and `ISPI` abstractions that you implement for your hardware.

## Features

- **TemperatureSensor** (BME280) — Temperature, humidity, and pressure via I2C
- **Accelerometer** (ADXL345) — 3-axis acceleration via SPI
- **PressureSensor** (BMP280) — Temperature and pressure via I2C
- Non-copyable, movable driver objects
- Factory calibration read during `init()` with datasheet-accurate integer compensation
- All operations return `sensor::Status` for explicit error handling
- Strict compiler warnings (`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`)

## Architecture

```
Concrete drivers (sensor::drivers::)
        │
    ISensor interface (sensor::)
        │
HAL interfaces — II2C / ISPI (sensor::)
```

- **Concrete drivers** — each driver owns a reference to its bus interface, reads calibration on `init()`, and exposes sensor-specific methods.
- **`ISensor`** — abstract base with common lifecycle: `init()`, `reset()`, `isConnected()`, `getDeviceId()`, `getName()`.
- **HAL interfaces** — `II2C` and `ISPI` are pure-virtual; platform code implements them, tests use GoogleMock.

Bus interfaces are passed by reference — the caller owns the bus object's lifetime.

## Project Structure

```
├── CMakeLists.txt
├── include/
│   ├── sensor_base.hpp            # ISensor interface + Status enum
│   ├── i2c_interface.hpp          # II2C pure-virtual
│   ├── spi_interface.hpp          # ISPI pure-virtual
│   └── drivers/
│       ├── temperature_sensor.hpp # BME280
│       ├── accelerometer.hpp      # ADXL345
│       └── pressure_sensor.hpp    # BMP280
├── src/drivers/
│   ├── temperature_sensor.cpp
│   ├── accelerometer.cpp
│   └── pressure_sensor.cpp
├── tests/
│   ├── test_temperature_sensor.cpp
│   ├── test_accelerometer.cpp
│   └── test_pressure_sensor.cpp
├── examples/
│   └── main.cpp
└── docs/
    ├── api_reference.md
    └── specs/                     # Driver specs extracted from datasheets
```

## Build

```bash
# Configure and build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Run all tests
cd build && ctest --output-on-failure

# Build without tests
cmake -B build -DBUILD_TESTS=OFF
cmake --build build
```

Requires a C++17-compatible compiler and CMake 3.14+.

## Usage

Implement the bus interface for your platform, then instantiate a driver:

```cpp
#include "i2c_interface.hpp"
#include "drivers/temperature_sensor.hpp"

// Implement II2C for your platform
class MyI2C : public sensor::II2C { /* ... */ };

int main() {
    MyI2C i2c;
    sensor::drivers::TemperatureSensor bme280(i2c, 0x76);

    if (bme280.init() == sensor::Status::Ok) {
        float temp;
        if (bme280.readTemperature(temp) == sensor::Status::Ok) {
            // use temp
        }
    }
}
```

### Driver APIs

| Driver | Key Methods |
|---|---|
| **TemperatureSensor** | `readTemperature(float&)`, `readAll(float& temp, float& press, float& hum)`, `setMode()` |
| **Accelerometer** | `readAcceleration(float& x, float& y, float& z)`, `setRange(Range)`, `setDataRate(DataRate)` |
| **PressureSensor** | `readPressure(float&)`, `readTemperature(float&)`, `readAll(float& temp, float& press)`, `setMode()` |

All methods return `sensor::Status`. Always check the return value.

## Testing

Tests use **GoogleTest v1.14 + GoogleMock**, fetched automatically via CMake `FetchContent`. The test suite includes 89 tests across three test binaries:

```bash
# Run all tests
cd build && ctest --output-on-failure

# Run a single test binary
./build/test_temperature
./build/test_accelerometer
./build/test_pressure

# Run a specific test case
./build/test_temperature --gtest_filter='TemperatureSensorTest.ReadTemperature_Success'
```

Each driver's test file follows the same pattern:

1. A `Mock*` class for the bus interface (e.g., `MockI2C`, `MockSPI`)
2. A `::testing::Test` fixture holding the mock and driver instance
3. Helper methods for common expectations (`expectChipIdRead`, `expectCalibration`, `initSuccessfully`)

## Documentation

- [`docs/api_reference.md`](docs/api_reference.md) — Full API reference
- [`docs/specs/`](docs/specs/) — Driver specifications extracted from datasheets (BME280, BMP280, ADXL345)

## License

MIT License. See [LICENSE](LICENSE) for details.
