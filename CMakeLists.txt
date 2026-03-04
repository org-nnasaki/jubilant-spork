cmake_minimum_required(VERSION 3.16)
project(EmbeddedSensorLib VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Strict compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)

# Library
add_library(sensor_drivers
    src/drivers/temperature_sensor.cpp
    src/drivers/accelerometer.cpp
    src/drivers/pressure_sensor.cpp
)
target_include_directories(sensor_drivers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Tests
option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()

    # Temperature sensor tests
    add_executable(test_temperature tests/test_temperature_sensor.cpp)
    target_link_libraries(test_temperature PRIVATE sensor_drivers GTest::gtest_main GTest::gmock)
    add_test(NAME TemperatureSensorTests COMMAND test_temperature)

    # Accelerometer tests
    add_executable(test_accelerometer tests/test_accelerometer.cpp)
    target_link_libraries(test_accelerometer PRIVATE sensor_drivers GTest::gtest_main GTest::gmock)
    add_test(NAME AccelerometerTests COMMAND test_accelerometer)

    # Pressure sensor tests
    add_executable(test_pressure tests/test_pressure_sensor.cpp)
    target_link_libraries(test_pressure PRIVATE sensor_drivers GTest::gtest_main GTest::gmock)
    add_test(NAME PressureSensorTests COMMAND test_pressure)
endif()
