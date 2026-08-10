#ifndef ESP_DRONE_I2C_DRV_H
#define ESP_DRONE_I2C_DRV_H

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define I2C_NO_INTERNAL_ADDRESS 0xFFFF
#define I2C_7BIT_ADDRESS_COUNT 128

typedef struct {
    i2c_port_num_t i2cPort;
    uint32_t i2cClockSpeed;
    gpio_num_t gpioSCLPin;
    gpio_num_t gpioSDAPin;
    bool enableInternalPullup;
} I2cDef;

typedef struct {
    const I2cDef *def;
    SemaphoreHandle_t isBusFreeMutex;
    i2c_master_bus_handle_t busHandle;
    i2c_master_dev_handle_t deviceHandles[I2C_7BIT_ADDRESS_COUNT];
    bool isInitialized;
} I2cDrv;

// Definitions of i2c busses found in c file.
extern I2cDrv deckBus;
extern I2cDrv sensorsBus;

/**
 * Initialize i2c peripheral as defined by static I2cDef structs.
 */
bool i2cdrvInit(I2cDrv *i2c);

#endif

