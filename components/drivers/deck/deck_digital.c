/*
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2015 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * digital.c - Deck-API digital IO implementation
 */

//#include "deck.h"
#include "driver/gpio.h"
#include "deck_digital.h"
#include "config.h"

void pinMode(uint32_t pin, uint32_t mode)
{
#ifdef TARGET_MCU_ESP32
    if (pin > 35) {
#elif defined(TARGET_MCU_ESP32S2)
    if (pin > 43) {
#endif
        return;
    }

    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << pin);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings

    //set as output mode
    if (mode == OUTPUT) {
        io_conf.mode = GPIO_MODE_OUTPUT;
    }

    if (mode == INPUT_PULLUP) {
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = 1;
    }

    if (mode == INPUT_PULLDOWN) {
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = 1;
    }

    gpio_config(&io_conf);
}

void digitalWrite(uint32_t pin, uint32_t val)
{
#ifdef TARGET_MCU_ESP32
    if (pin > 35) {
#elif defined(TARGET_MCU_ESP32S2)
    if (pin > 43) {
#endif
        return;
    }

    if (val) {
        val = 1;
    }

    gpio_set_level(pin, val);
}

int digitalRead(uint32_t pin)
{
#ifdef TARGET_MCU_ESP32
    if (pin > 35) {
#elif defined(TARGET_MCU_ESP32S2)
    if (pin > 43) {
#endif
        return 0;
    }

    int val = gpio_get_level(pin);
    return val;
}
