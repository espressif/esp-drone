/**
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (c) 2014, Bitcraze AB, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 *
 * i2c_drv.c - i2c driver implementation
 *
 * @note
 * For some reason setting CR1 reg in sequence with
 * I2C_AcknowledgeConfig(I2C_SENSORS, ENABLE) and after
 * I2C_GenerateSTART(I2C_SENSORS, ENABLE) sometimes creates an
 * instant start->stop condition (3.9us long) which I found out with an I2C
 * analyzer. This fast start->stop is only possible to generate if both
 * start and stop flag is set in CR1 at the same time. So i tried setting the CR1
 * at once with I2C_SENSORS->CR1 = (I2C_CR1_START | I2C_CR1_ACK | I2C_CR1_PE) and the
 * problem is gone. Go figure...
 */
#include <inttypes.h>

#include "esp_err.h"
#include "i2c_drv.h"
#include "config.h"
#define DEBUG_MODULE "I2CDRV"
#include "debug_cf.h"

// Definitions of sensors I2C bus
#define I2C_DEFAULT_SENSORS_CLOCK_SPEED             400000

// Definition of eeprom and deck I2C buss,use two i2c with 400Khz clock simultaneously could trigger the watchdog
#define I2C_DEFAULT_DECK_CLOCK_SPEED                100000

// Cost definitions of busses
static const I2cDef sensorBusDef = {
    .i2cPort            = I2C_NUM_0,
    .i2cClockSpeed      = I2C_DEFAULT_SENSORS_CLOCK_SPEED,
    .gpioSCLPin         = CONFIG_I2C0_PIN_SCL,
    .gpioSDAPin         = CONFIG_I2C0_PIN_SDA,
    .enableInternalPullup = false,
};

I2cDrv sensorsBus = {
    .def                = &sensorBusDef,
};

static const I2cDef deckBusDef = {
    .i2cPort            = I2C_NUM_1,
    .i2cClockSpeed      = I2C_DEFAULT_DECK_CLOCK_SPEED,
    .gpioSCLPin         = CONFIG_I2C1_PIN_SCL,
    .gpioSDAPin         = CONFIG_I2C1_PIN_SDA,
    .enableInternalPullup = true,
};

I2cDrv deckBus = {
    .def                = &deckBusDef,
};

bool i2cdrvInit(I2cDrv *i2c)
{
    if (i2c == NULL || i2c->def == NULL) {
        return false;
    }
    if (i2c->isInitialized) {
        return true;
    }

    i2c->isBusFreeMutex = xSemaphoreCreateMutex();
    if (i2c->isBusFreeMutex == NULL) {
        DEBUG_PRINTW("I2C %d mutex allocation failed", i2c->def->i2cPort);
        return false;
    }

    const i2c_master_bus_config_t busConfig = {
        .i2c_port = i2c->def->i2cPort,
        .sda_io_num = i2c->def->gpioSDAPin,
        .scl_io_num = i2c->def->gpioSCLPin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = i2c->def->enableInternalPullup,
    };
    const esp_err_t err = i2c_new_master_bus(&busConfig, &i2c->busHandle);
    if (err != ESP_OK) {
        vSemaphoreDelete(i2c->isBusFreeMutex);
        i2c->isBusFreeMutex = NULL;
        i2c->busHandle = NULL;
        DEBUG_PRINTW("I2C %d master bus init failed: %s", i2c->def->i2cPort, esp_err_to_name(err));
        return false;
    }

    i2c->isInitialized = true;
    DEBUG_PRINTI("I2C %d master bus initialized (SDA=%d, SCL=%d, %" PRIu32 " Hz)",
                 i2c->def->i2cPort, i2c->def->gpioSDAPin,
                 i2c->def->gpioSCLPin, i2c->def->i2cClockSpeed);
    return true;
}
