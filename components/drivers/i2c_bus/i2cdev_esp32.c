/**
 * ESP-Drone Firmware
 *
 * I2C device compatibility layer backed by the ESP-IDF master bus/device API.
 */
#define DEBUG_MODULE "I2CDEV"

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "i2cdev.h"

static bool transferArgumentsAreValid(I2C_Dev *dev, uint8_t devAddress,
                                      uint16_t len, const uint8_t *data)
{
    return dev != NULL && devAddress < I2C_7BIT_ADDRESS_COUNT &&
           len > 0 && data != NULL;
}

static bool lockDevice(I2C_Dev *dev, uint8_t devAddress,
                       i2c_master_dev_handle_t *deviceHandle)
{
    if (deviceHandle == NULL || devAddress >= I2C_7BIT_ADDRESS_COUNT ||
        !i2cdrvInit(dev)) {
        return false;
    }

    if (xSemaphoreTake(dev->isBusFreeMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
        return false;
    }

    if (dev->deviceHandles[devAddress] == NULL) {
        const i2c_device_config_t deviceConfig = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = devAddress,
            .scl_speed_hz = dev->def->i2cClockSpeed,
        };
        const esp_err_t err = i2c_master_bus_add_device(dev->busHandle, &deviceConfig,
                                                        &dev->deviceHandles[devAddress]);
        if (err != ESP_OK) {
            xSemaphoreGive(dev->isBusFreeMutex);
            return false;
        }
    }

    *deviceHandle = dev->deviceHandles[devAddress];
    return true;
}

static void unlockDevice(I2C_Dev *dev)
{
    xSemaphoreGive(dev->isBusFreeMutex);
}

static bool readRegister(I2C_Dev *dev, uint8_t devAddress,
                         const uint8_t *registerAddress, size_t registerAddressSize,
                         uint16_t len, uint8_t *data)
{
    if (!transferArgumentsAreValid(dev, devAddress, len, data)) {
        return false;
    }

    i2c_master_dev_handle_t deviceHandle = NULL;
    if (!lockDevice(dev, devAddress, &deviceHandle)) {
        return false;
    }

    esp_err_t err;
    if (registerAddressSize == 0) {
        err = i2c_master_receive(deviceHandle, data, len, I2C_TIMEOUT_MS);
    } else {
        err = i2c_master_transmit_receive(deviceHandle, registerAddress,
                                          registerAddressSize, data, len,
                                          I2C_TIMEOUT_MS);
    }

    unlockDevice(dev);
    return err == ESP_OK;
}

static bool writeRegister(I2C_Dev *dev, uint8_t devAddress,
                          const uint8_t *registerAddress, size_t registerAddressSize,
                          uint16_t len, const uint8_t *data)
{
    if (!transferArgumentsAreValid(dev, devAddress, len, data)) {
        return false;
    }

    i2c_master_dev_handle_t deviceHandle = NULL;
    if (!lockDevice(dev, devAddress, &deviceHandle)) {
        return false;
    }

    esp_err_t err;
    if (registerAddressSize == 0) {
        err = i2c_master_transmit(deviceHandle, data, len, I2C_TIMEOUT_MS);
    } else {
        i2c_master_transmit_multi_buffer_info_t buffers[] = {
            {
                .write_buffer = registerAddress,
                .buffer_size = registerAddressSize,
            },
            {
                .write_buffer = data,
                .buffer_size = len,
            },
        };
        err = i2c_master_multi_buffer_transmit(deviceHandle, buffers,
                                               sizeof(buffers) / sizeof(buffers[0]),
                                               I2C_TIMEOUT_MS);
    }

    unlockDevice(dev);
    return err == ESP_OK;
}

bool i2cdevInit(I2C_Dev *dev)
{
    return i2cdrvInit(dev);
}

bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint16_t len, uint8_t *data)
{
    return readRegister(dev, devAddress, NULL, 0, len, data);
}

bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t *data)
{
    return i2cdevReadReg8(dev, devAddress, memAddress, 1, data);
}

bool i2cdevReadBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                   uint8_t bitNum, uint8_t *data)
{
    if (data == NULL || bitNum > 7) {
        return false;
    }

    uint8_t byte = 0;
    if (!i2cdevReadByte(dev, devAddress, memAddress, &byte)) {
        return false;
    }
    *data = byte & (1U << bitNum);
    return true;
}

bool i2cdevReadBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data)
{
    if (data == NULL || bitStart > 7 || length == 0 || length > 8 ||
        length > bitStart + 1) {
        return false;
    }

    uint8_t byte = 0;
    if (!i2cdevReadByte(dev, devAddress, memAddress, &byte)) {
        return false;
    }
    const uint8_t shift = bitStart - length + 1;
    const uint8_t mask = (uint8_t)(((1U << length) - 1U) << shift);
    *data = (byte & mask) >> shift;
    return true;
}

bool i2cdevReadReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint16_t len, uint8_t *data)
{
    if (memAddress == I2CDEV_NO_MEM_ADDR) {
        return readRegister(dev, devAddress, NULL, 0, len, data);
    }
    return readRegister(dev, devAddress, &memAddress, sizeof(memAddress), len, data);
}

bool i2cdevReadReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                     uint16_t len, uint8_t *data)
{
    if (memAddress == I2C_NO_INTERNAL_ADDRESS) {
        return readRegister(dev, devAddress, NULL, 0, len, data);
    }
    const uint8_t address[] = {
        (uint8_t)(memAddress >> 8),
        (uint8_t)memAddress,
    };
    return readRegister(dev, devAddress, address, sizeof(address), len, data);
}

bool i2cdevWrite(I2C_Dev *dev, uint8_t devAddress, uint16_t len, uint8_t *data)
{
    return writeRegister(dev, devAddress, NULL, 0, len, data);
}

bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t data)
{
    return i2cdevWriteReg8(dev, devAddress, memAddress, 1, &data);
}

bool i2cdevWriteBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data)
{
    if (bitNum > 7) {
        return false;
    }

    uint8_t byte = 0;
    if (!i2cdevReadByte(dev, devAddress, memAddress, &byte)) {
        return false;
    }
    byte = data != 0 ? byte | (1U << bitNum) : byte & ~(1U << bitNum);
    return i2cdevWriteByte(dev, devAddress, memAddress, byte);
}

bool i2cdevWriteBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitStart, uint8_t length, uint8_t data)
{
    if (bitStart > 7 || length == 0 || length > 8 || length > bitStart + 1) {
        return false;
    }

    uint8_t byte = 0;
    if (!i2cdevReadByte(dev, devAddress, memAddress, &byte)) {
        return false;
    }
    const uint8_t shift = bitStart - length + 1;
    const uint8_t mask = (uint8_t)(((1U << length) - 1U) << shift);
    data = (data << shift) & mask;
    byte = (byte & ~mask) | data;
    return i2cdevWriteByte(dev, devAddress, memAddress, byte);
}

bool i2cdevWriteReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint16_t len, uint8_t *data)
{
    if (memAddress == I2CDEV_NO_MEM_ADDR) {
        return writeRegister(dev, devAddress, NULL, 0, len, data);
    }
    return writeRegister(dev, devAddress, &memAddress, sizeof(memAddress), len, data);
}

bool i2cdevWriteReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                      uint16_t len, uint8_t *data)
{
    if (memAddress == I2C_NO_INTERNAL_ADDRESS) {
        return writeRegister(dev, devAddress, NULL, 0, len, data);
    }
    const uint8_t address[] = {
        (uint8_t)(memAddress >> 8),
        (uint8_t)memAddress,
    };
    return writeRegister(dev, devAddress, address, sizeof(address), len, data);
}
