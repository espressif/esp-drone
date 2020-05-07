/**
 * ESPlane Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * adc.c - Analog Digital Conversion
 *
 *
 */

#include "driver/adc.h"
//#include "esp_adc_cal.h"//adc not supported in s2 yet

#include "adc_esp32.h"
#include "pm_esplane.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "ADC"
#include "debug_cf.h"

static bool isInit;

// static esp_adc_cal_characteristics_t *adc_chars;
// static const adc_channel_t channel = CONFIG_ADC1_PIN; //GPIO34 if ADC1, GPIO14 if ADC2 ,ADC1 channel 7 is GPIO35
// static const adc_atten_t atten = ADC_ATTEN_DB_11;   //11dB attenuation (ADC_ATTEN_DB_11) gives full-scale voltage 3.9V
// static const adc_unit_t unit = ADC_UNIT_1;
// #define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate

static void checkEfuse()
{
    // //Check TP is burned into eFuse
    // if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
    //     DEBUG_PRINT_LOCAL("eFuse Two Point: Supported\n");
    // } else {
    //     DEBUG_PRINT_LOCAL("eFuse Two Point: NOT supported\n");
    // }

    // //Check Vref is burned into eFuse
    // if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
    //     DEBUG_PRINT_LOCAL("eFuse Vref: Supported\n");
    // } else {
    //     DEBUG_PRINT_LOCAL("eFuse Vref: NOT supported\n");
    // }
}

float analogReadVoltage(uint32_t pin)
{
         float voltage = 2.0;
    // DEBUG_PRINTD("get analog Voltage !");
    // int adc_reading = adc1_get_raw((adc1_channel_t)channel);
    // voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
         return voltage / 1000.0;
}

void adcInit(void)
{

    if (isInit) {
        return;
    }

    // checkEfuse();
    // //Configure ADC
    // if (unit == ADC_UNIT_1) {
    //     adc1_config_width(ADC_WIDTH_BIT_12);
    //     adc1_config_channel_atten(channel, atten);
    // } else {
    //     adc2_config_channel_atten((adc2_channel_t)channel, atten);
    // }

    // //Characterize ADC
    // adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    // esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    DEBUG_PRINTI("adc not supported in s2 yet !");

    isInit = true;
}

bool adcTest(void)
{
    return isInit;
}
