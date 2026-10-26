# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(__PROJECT_GROUP_LINK_COMPONENTS ON)

list(APPEND EXTRA_COMPONENT_DIRS
     "${CMAKE_CURRENT_LIST_DIR}/components/core/crazyflie"
     "${CMAKE_CURRENT_LIST_DIR}/components/drivers/deck"
     "${CMAKE_CURRENT_LIST_DIR}/components/drivers/general/adc"
     "${CMAKE_CURRENT_LIST_DIR}/components/drivers/general/buzzer"
     "${CMAKE_CURRENT_LIST_DIR}/components/drivers/general/led"
     "${CMAKE_CURRENT_LIST_DIR}/components/drivers/general/motors"
     "${CMAKE_CURRENT_LIST_DIR}/components/drivers/general/wifi"
     "${CMAKE_CURRENT_LIST_DIR}/components/drivers/i2c_bus"
     "${CMAKE_CURRENT_LIST_DIR}/components/drivers/i2c_devices/eeprom"
     "${CMAKE_CURRENT_LIST_DIR}/components/drivers/i2c_devices/hmc5883l"
     "${CMAKE_CURRENT_LIST_DIR}/components/drivers/i2c_devices/mpu6050"
     "${CMAKE_CURRENT_LIST_DIR}/components/drivers/i2c_devices/ms5611"
     "${CMAKE_CURRENT_LIST_DIR}/components/drivers/i2c_devices/vl53l0"
     "${CMAKE_CURRENT_LIST_DIR}/components/drivers/i2c_devices/vl53l1"
     "${CMAKE_CURRENT_LIST_DIR}/components/drivers/spi_devices/pmw3901"
     "${CMAKE_CURRENT_LIST_DIR}/components/lib/dsp_lib")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESPDrone)

if(NOT ((IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR EQUAL 5) OR
        (IDF_VERSION_MAJOR EQUAL 6 AND IDF_VERSION_MINOR EQUAL 1)))
    message(FATAL_ERROR
        "ESP-Drone supports ESP-IDF release/v5.5 and release/v6.1 only; "
        "detected ${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH}")
endif()
