#pragma once

#include <cstdint>
#include <string>

namespace sensor {

enum class Status : uint8_t {
    Ok = 0,
    Error,
    Timeout,
    InvalidParam,
    NotInitialized,
    DeviceNotFound
};

class ISensor {
public:
    virtual ~ISensor() = default;

    ISensor(const ISensor&) = delete;
    ISensor& operator=(const ISensor&) = delete;
    ISensor(ISensor&&) = default;
    ISensor& operator=(ISensor&&) = default;

    virtual Status init() = 0;
    virtual Status reset() = 0;
    virtual bool isConnected() = 0;
    virtual uint8_t getDeviceId() = 0;
    virtual std::string getName() const = 0;

protected:
    ISensor() = default;
};

} // namespace sensor
