/**
 * ESP-Drone Firmware
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

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "adc_esp32.h"
#include "config.h"
#include "pm_esplane.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "ADC"
#include "debug_cf.h"

static bool isInit;

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle;
static bool calibration_enabled;
#ifdef CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_7; //GPIO35 if ADC1
#else
static const adc_channel_t channel = ADC_CHANNEL_1;     // GPIO2 if ADC1
#endif

static const adc_bitwidth_t width = ADC_BITWIDTH_DEFAULT;
static const adc_atten_t atten = ADC_ATTEN_DB_12;
static const adc_unit_t unit = ADC_UNIT_1;
#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   30          //Multisampling

static bool initCalibration(void)
{
    esp_err_t result = ESP_ERR_NOT_SUPPORTED;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = width,
    };
    result = adc_cali_create_scheme_curve_fitting(&config, &adc_cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = width,
#ifdef CONFIG_IDF_TARGET_ESP32
        .default_vref = DEFAULT_VREF,
#endif
    };
    result = adc_cali_create_scheme_line_fitting(&config, &adc_cali_handle);
#endif

    if (result != ESP_OK) {
        DEBUG_PRINTW("ADC calibration unavailable: %s\n", esp_err_to_name(result));
    }
    return result == ESP_OK;
}

float analogReadVoltage(uint32_t pin)
{
    (void)pin;
    if (!isInit) {
        return 0.0f;
    }

    uint32_t adc_reading = 0;
    int valid_samples = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        int raw = 0;
        if (adc_oneshot_read(adc_handle, channel, &raw) == ESP_OK) {
            adc_reading += raw;
            valid_samples++;
        }
    }
    if (valid_samples == 0) {
        return 0.0f;
    }

    adc_reading /= valid_samples;
    int voltage_mv = 0;
    if (calibration_enabled && adc_cali_raw_to_voltage(adc_cali_handle, adc_reading, &voltage_mv) == ESP_OK) {
        return voltage_mv / 1000.0f;
    }

    // Conservative fallback when the target has no usable eFuse calibration.
    return (adc_reading * 3.1f) / 4095.0f;
}

void adcInit(void)
{
    if (isInit) {
        return;
    }

    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t result = adc_oneshot_new_unit(&unit_config, &adc_handle);
    if (result != ESP_OK) {
        DEBUG_PRINTE("ADC unit init failed: %s\n", esp_err_to_name(result));
        return;
    }

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = atten,
        .bitwidth = width,
    };
    result = adc_oneshot_config_channel(adc_handle, channel, &channel_config);
    if (result != ESP_OK) {
        DEBUG_PRINTE("ADC channel init failed: %s\n", esp_err_to_name(result));
        adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
        return;
    }

    calibration_enabled = initCalibration();

    isInit = true;
}

bool adcTest(void)
{
    return isInit;
}
