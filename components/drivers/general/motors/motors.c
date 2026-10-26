/**
 *
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
 * motors.c - Motor driver
 *
 */

#include <stdbool.h>
#include <inttypes.h>

#include "esp_err.h"

//FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stm32_legacy.h"
#include "motors.h"
#include "pm_esplane.h"
#include "log.h"
#define DEBUG_MODULE "MOTORS"
#include "debug_cf.h"

static uint16_t motorsConvBitsTo16(uint16_t bits);
static uint16_t motorsConv16ToBits(uint16_t bits);

uint32_t motor_ratios[] = {0, 0, 0, 0};

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec);
void motorsPlayMelody(uint16_t *notes);
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

const MotorPerifDef **motorMap; /* Current map configuration */

const uint32_t MOTORS[] = {MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4};

const uint16_t testsound[NBR_OF_MOTORS] = {A4, A5, F5, D5};

static bool isInit = false;
static bool isTimerInit = false;
static bool isHealthy = false;

ledc_channel_config_t motors_channel[NBR_OF_MOTORS] = {
    {
        .channel = MOT_PWM_CH1,
        .duty = 0,
        .gpio_num = MOTOR1_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH2,
        .duty = 0,
        .gpio_num = MOTOR2_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH3,
        .duty = 0,
        .gpio_num = MOTOR3_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = MOT_PWM_CH4,
        .duty = 0,
        .gpio_num = MOTOR4_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
};
/* Private functions */

static uint16_t motorsConvBitsTo16(uint16_t bits)
{
    return ((bits) << (16 - MOTORS_PWM_BITS));
}

static uint16_t motorsConv16ToBits(uint16_t bits)
{
    return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

static bool pwmTimerInit(void)
{
    if (isTimerInit) {
        // First to init will configure it
        return TRUE;
    }

    /*
     * Prepare and set configuration of timers
     * that will be used by MOTORS Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = MOTORS_PWM_BITS, // resolution of PWM duty
        .freq_hz = 15000,					// frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE, // timer mode
        .timer_num = LEDC_TIMER_0,			// timer index
        .clk_cfg = LEDC_AUTO_CLK,
    };

    // Set configuration of timer0 for high speed channels
    if (ledc_timer_config(&ledc_timer) == ESP_OK) {
        isTimerInit = TRUE;
        return TRUE;
    }

    return FALSE;
}

/* Public functions */

//Initialization. Will set all motors ratio to 0%
bool motorsInit(const MotorPerifDef **motorMapSelect)
{
    if (isInit) {
        return isHealthy;
    }
    if (motorMapSelect == NULL) {
        return false;
    }
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        if (motorMapSelect[i] == NULL) {
            return false;
        }
    }

    motorMap = motorMapSelect;

    if (!pwmTimerInit()) {
        return false;
    }

    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        esp_err_t err = ledc_channel_config(&motors_channel[i]);
        if (err == ESP_OK) {
            err = ledc_set_duty(motors_channel[i].speed_mode,
                                motors_channel[i].channel, 0);
        }
        if (err == ESP_OK) {
            err = ledc_update_duty(motors_channel[i].speed_mode,
                                   motors_channel[i].channel);
        }
        if (err != ESP_OK) {
            DEBUG_PRINTW("Motor %d PWM init failed: %s", i, esp_err_to_name(err));
            for (int configured = 0; configured <= i; configured++) {
                ledc_stop(motors_channel[configured].speed_mode,
                          motors_channel[configured].channel, 0);
            }
            motorMap = NULL;
            return false;
        }
    }

    isInit = true;
    isHealthy = true;
    return true;
}

void motorsDeInit(const MotorPerifDef **motorMapSelect)
{
    (void)motorMapSelect;
    if (!isInit) {
        return;
    }
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        ledc_stop(motors_channel[i].speed_mode, motors_channel[i].channel, 0);
        motor_ratios[i] = 0;
    }
    isInit = false;
    isHealthy = false;
    motorMap = NULL;
}

/* Bypass the health guard so a fault can still force every PWM output low. */
static void motorsForceStopAll(void)
{
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        ledc_stop(motors_channel[i].speed_mode, motors_channel[i].channel, 0);
        motor_ratios[i] = 0;
    }
}

bool motorsTest(void)
{
    if (!isInit || !isHealthy || motorMap == NULL) {
        return false;
    }

#ifdef CONFIG_MOTORS_STARTUP_TEST
    for (size_t i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++) {
        if (motorMap[i]->drvType == BRUSHED) {
#ifdef ACTIVATE_STARTUP_SOUND
            motorsBeep(MOTORS[i], true, testsound[i], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4) / 20);
            vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
            motorsBeep(MOTORS[i], false, 0, 0);
            vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#else
            if (!motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO)) {
                return false;
            }
            vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
            if (!motorsSetRatio(MOTORS[i], 0)) {
                return false;
            }
            vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#endif
        }
    }
#endif

    return isHealthy;
}

// Ithrust is thrust mapped for 65536 <==> 60 grams
bool motorsSetRatio(uint32_t id, uint16_t ithrust)
{
    if (!isInit || !isHealthy || id >= NBR_OF_MOTORS) {
        return false;
    }

    uint16_t ratio;

    ratio = ithrust;

#ifdef ENABLE_THRUST_BAT_COMPENSATED

    if (motorMap[id]->drvType == BRUSHED) {
        float thrust = ((float)ithrust / 65536.0f) * 40; //根据实际重量修改
        float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
        float supply_voltage = pmGetBatteryVoltage();
        if (supply_voltage <= 0.1f) {
            motorsForceStopAll();
            isHealthy = false;
            return false;
        }
        float percentage = volts / supply_voltage;
        percentage = percentage > 1.0f ? 1.0f : percentage;
        ratio = percentage * UINT16_MAX;
    }

#endif
    esp_err_t err = ledc_set_duty(motors_channel[id].speed_mode,
                                  motors_channel[id].channel,
                                  (uint32_t)motorsConv16ToBits(ratio));
    if (err == ESP_OK) {
        err = ledc_update_duty(motors_channel[id].speed_mode,
                               motors_channel[id].channel);
    }
    if (err != ESP_OK) {
        motorsForceStopAll();
        isHealthy = false;
        DEBUG_PRINTW("Motor %"PRIu32" PWM update failed: %s", id, esp_err_to_name(err));
        return false;
    }
    motor_ratios[id] = ratio;
#ifdef DEBUG_EP2
    DEBUG_PRINT_LOCAL("motors ID = %d ,ithrust_10bit = %d", id, (uint32_t)motorsConv16ToBits(ratio));
#endif
    return true;
}

int motorsGetRatio(uint32_t id)
{
    if (!isInit || id >= NBR_OF_MOTORS) {
        return -1;
    }
    return motorsConvBitsTo16((uint16_t)ledc_get_duty(motors_channel[id].speed_mode,
                                                      motors_channel[id].channel));
}

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
    uint32_t freq_hz = 15000;
    if (!isInit || !isHealthy || id < 0 || id >= NBR_OF_MOTORS ||
        (enable && frequency == 0)) {
        return;
    }
    if (ratio != 0) {
        ratio = (uint16_t)(0.05*(1<<16));
    }
    
    if (enable) {
        freq_hz = frequency;
    }
    
    if (ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freq_hz) == 0 ||
        ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel,
                      (uint32_t)motorsConv16ToBits(ratio)) != ESP_OK ||
        ledc_update_duty(motors_channel[id].speed_mode,
                         motors_channel[id].channel) != ESP_OK) {
        motorsForceStopAll();
        isHealthy = false;
    }
}

// Play a tone with a given frequency and a specific duration in milliseconds (ms)
void motorsPlayTone(uint16_t frequency, uint16_t duration_msec)
{
    motorsBeep(MOTOR_M1, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(MOTOR_M2, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(MOTOR_M3, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(MOTOR_M4, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    vTaskDelay(M2T(duration_msec));
    motorsBeep(MOTOR_M1, false, frequency, 0);
    motorsBeep(MOTOR_M2, false, frequency, 0);
    motorsBeep(MOTOR_M3, false, frequency, 0);
    motorsBeep(MOTOR_M4, false, frequency, 0);
}

// Plays a melody from a note array
void motorsPlayMelody(uint16_t *notes)
{
    int i = 0;
    uint16_t note;     // Note in hz
    uint16_t duration; // Duration in ms

    do
    {
      note = notes[i++];
      duration = notes[i++];
      motorsPlayTone(note, duration);
    } while (duration != 0);
}
LOG_GROUP_START(pwm)
LOG_ADD(LOG_UINT32, m1_pwm, &motor_ratios[0])
LOG_ADD(LOG_UINT32, m2_pwm, &motor_ratios[1])
LOG_ADD(LOG_UINT32, m3_pwm, &motor_ratios[2])
LOG_ADD(LOG_UINT32, m4_pwm, &motor_ratios[3])
LOG_GROUP_STOP(pwm)
