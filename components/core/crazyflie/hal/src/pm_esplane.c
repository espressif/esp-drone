/**
*
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
 * pm.c - Power Management driver and functions.
 */

#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "config.h"
#include "system.h"
#include "pm_esplane.h"
#include "adc_esp32.h"
#include "led.h"
#include "log.h"
#include "ledseq.h"
#include "commander.h"
#include "sound.h"
#include "stm32_legacy.h"
//#include "deck.h"
#define DEBUG_MODULE "PM"
#include "debug_cf.h"
typedef struct _PmSyslinkInfo {
    union {
        uint8_t flags;
        struct {
            uint8_t chg : 1;
            uint8_t pgood : 1;
            uint8_t unused : 6;
        };
    };
    float vBat;
    float chargeCurrent;
} __attribute__((packed)) PmSyslinkInfo;

static float batteryVoltage;
static uint16_t batteryVoltageMV;
static float batteryVoltageMin = 6.0;
static float batteryVoltageMax = 0.0;

static float extBatteryVoltage;
static uint16_t extBatteryVoltageMV;
static uint8_t extBatVoltDeckPin;
static float extBatVoltMultiplier;
static float extBatteryCurrent;
static uint8_t extBatCurrDeckPin;
static float extBatCurrAmpPerVolt;

static uint32_t batteryLowTimeStamp;
static uint32_t batteryCriticalLowTimeStamp;
static bool isInit;
static PMStates pmState;
static PmSyslinkInfo pmSyslinkInfo;

static uint8_t batteryLevel;

static void pmSetBatteryVoltage(float voltage);

const static float bat671723HS25C[10] = {
    3.00, // 00%
    3.78, // 10%
    3.83, // 20%
    3.87, // 30%
    3.89, // 40%
    3.92, // 50%
    3.96, // 60%
    4.00, // 70%
    4.04, // 80%
    4.10  // 90%
};

void pmInit(void)
{
    if (isInit) {
        return;
    }

    pmEnableExtBatteryVoltMeasuring(35, 2);

    //TODO:
    pmSyslinkInfo.vBat = 3.7f;
    pmSetBatteryVoltage(pmSyslinkInfo.vBat);

    pmSyslinkInfo.pgood = false;
    pmSyslinkInfo.chg = false;

    xTaskCreate(pmTask, PM_TASK_NAME, PM_TASK_STACKSIZE, NULL, PM_TASK_PRI, NULL);
    isInit = true;

}

bool pmTest(void)
{
    return isInit;
}

/**
 * Sets the battery voltage and its min and max values during running
 */
static void pmSetBatteryVoltage(float voltage)
{
    batteryVoltage = voltage;
    batteryVoltageMV = (uint16_t)(voltage * 1000);

    if (batteryVoltageMax < voltage) {
        batteryVoltageMax = voltage;
    }

    if (batteryVoltageMin > voltage) {
        batteryVoltageMin = voltage;
    }
}

/**
 * Shutdown system
 */
static void pmSystemShutdown(void)
{
#ifdef ACTIVATE_AUTO_SHUTDOWN
//TODO: Implement syslink call to shutdown
#endif
}

/**
 * Returns a number from 0 to 9 where 0 is completely discharged
 * and 9 is 90% charged.
 */
static int32_t pmBatteryChargeFromVoltage(float voltage)
{
    int charge = 0;

    if (voltage < bat671723HS25C[0]) {
        return 0;
    }

    if (voltage > bat671723HS25C[9]) {
        return 9;
    }

    while (voltage > bat671723HS25C[charge]) {
        charge++;
    }

    return charge;
}

float pmGetBatteryVoltage(void)
{
    return batteryVoltage;
}

float pmGetBatteryVoltageMin(void)
{
    return batteryVoltageMin;
}

float pmGetBatteryVoltageMax(void)
{
    return batteryVoltageMax;
}

void pmSyslinkUpdate(SyslinkPacket *slp)
{
  if (slp->type == SYSLINK_PM_BATTERY_STATE) {
    memcpy(&pmSyslinkInfo, &slp->data[0], sizeof(pmSyslinkInfo));
    pmSetBatteryVoltage(pmSyslinkInfo.vBat);
  }
}

void pmSetChargeState(PMChargeStates chgState)
{
    // TODO: Send syslink packafe with charge state
}

PMStates pmUpdateState()
{
    PMStates state;
    bool isCharging = pmSyslinkInfo.chg;
    bool isPgood = pmSyslinkInfo.pgood;
    uint32_t batteryLowTime;

    batteryLowTime = xTaskGetTickCount() - batteryLowTimeStamp;

    if (isPgood && !isCharging) {
        state = charged;
    } else if (isPgood && isCharging) {
        state = charging;
    } else if (!isPgood && !isCharging && (batteryLowTime > PM_BAT_LOW_TIMEOUT)) {
        state = lowPower;
    } else {
        state = battery;
    }

    return state;
}

void pmEnableExtBatteryCurrMeasuring(uint8_t pin, float ampPerVolt)
{
    extBatCurrDeckPin = pin;
    extBatCurrAmpPerVolt = ampPerVolt;
}

float pmMeasureExtBatteryCurrent(void)
{
    float current;

    if (extBatCurrDeckPin) {
        current = analogReadVoltage(extBatCurrDeckPin) * extBatCurrAmpPerVolt;
    } else {
        current = 0.0;
    }

    return current;
}

void pmEnableExtBatteryVoltMeasuring(uint8_t pin, float multiplier)
{
    extBatVoltDeckPin = pin;
    extBatVoltMultiplier = multiplier;
    //TODO:init ad
}

float pmMeasureExtBatteryVoltage(void)
{
    float voltage;

    if (extBatVoltDeckPin) {
        voltage = analogReadVoltage(extBatVoltDeckPin) * extBatVoltMultiplier; //esplane *2
    } else {
        voltage = 0.0;
    }

    return voltage;
}

// return true if battery discharging
bool pmIsDischarging(void)
{
    PMStates pmState;
    pmState = pmUpdateState();
    return (pmState == lowPower) || (pmState == battery);
}

void pmTask(void *param)
{
    PMStates pmStateOld = battery;
    uint32_t tickCount = 0;

#ifdef configUSE_APPLICATION_TASK_TAG
	#if configUSE_APPLICATION_TASK_TAG == 1
    vTaskSetApplicationTaskTag(0, (void *)TASK_PM_ID_NBR);
    #endif
#endif

    tickCount = xTaskGetTickCount();
    batteryLowTimeStamp = tickCount;
    batteryCriticalLowTimeStamp = tickCount;
    pmSetChargeState(charge300mA);
    systemWaitStart();

    while (1) {
        vTaskDelay(M2T(100));
        tickCount = xTaskGetTickCount();

        extBatteryVoltage = pmMeasureExtBatteryVoltage();
        extBatteryVoltageMV = (uint16_t)(extBatteryVoltage * 1000);
        extBatteryCurrent = pmMeasureExtBatteryCurrent();
        pmSetBatteryVoltage(extBatteryVoltage);
        batteryLevel = pmBatteryChargeFromVoltage(pmGetBatteryVoltage()) * 10; //batteryLevel get from voltage %
#ifdef DEBUG_EP2
        DEBUG_PRINTD("batteryLevel=%u extBatteryVoltageMV=%u \n", batteryLevel, extBatteryVoltageMV);
#endif

        if (pmGetBatteryVoltage() > PM_BAT_LOW_VOLTAGE) {
            batteryLowTimeStamp = tickCount;
        }

        if (pmGetBatteryVoltage() > PM_BAT_CRITICAL_LOW_VOLTAGE) {
            batteryCriticalLowTimeStamp = tickCount;
        }

        pmState = pmUpdateState();

        if (pmState != pmStateOld) {
            // Actions on state change
            switch (pmState) {
                case charged:
                    soundSetEffect(SND_BAT_FULL);
                    systemSetCanFly(false);
                    DEBUG_PRINTI("charged \n");
                    break;

                case charging:
                    ledseqStop(LOWBAT_LED, seq_lowbat);
                    soundSetEffect(SND_USB_CONN);
                    systemSetCanFly(false);
                    DEBUG_PRINTI("charging \n");
                    break;

                case lowPower:
                    ledseqRun(LOWBAT_LED, seq_lowbat);
                    soundSetEffect(SND_BAT_LOW);
                    systemSetCanFly(true);
                    DEBUG_PRINTI("lowPower \n");
                    break;

                case battery:
                    soundSetEffect(SND_USB_DISC);
                    systemSetCanFly(true);
                    DEBUG_PRINTI("battery \n");
                    break;

                default:
                    systemSetCanFly(true);
                    break;
            }

            pmStateOld = pmState;
        }

        // Actions during state
        switch (pmState) {
            case charged:
                break;

            case charging: {
                uint32_t onTime;

                onTime = pmBatteryChargeFromVoltage(pmGetBatteryVoltage()) *
                         (LEDSEQ_CHARGE_CYCLE_TIME_500MA / 10);
                ledseqSetTimes(seq_charging, onTime, LEDSEQ_CHARGE_CYCLE_TIME_500MA - onTime);
            }
            break;

            case lowPower: {
                uint32_t batteryCriticalLowTime;

                batteryCriticalLowTime = tickCount - batteryCriticalLowTimeStamp;

                if (batteryCriticalLowTime > PM_BAT_CRITICAL_LOW_TIMEOUT) { //day
                    pmSystemShutdown();// shutdown when low battery
                }
            }
            break;

            case battery: {
                if ((commanderGetInactivityTime() > PM_SYSTEM_SHUTDOWN_TIMEOUT)) {
                    pmSystemShutdown();// shutdown when command not update
                }
            }
            break;

            default:
                break;
        }
    }
}

LOG_GROUP_START(pm)
LOG_ADD(LOG_FLOAT, vbat, &batteryVoltage)
LOG_ADD(LOG_UINT16, vbatMV, &batteryVoltageMV)
LOG_ADD(LOG_FLOAT, extVbat, &extBatteryVoltage)
LOG_ADD(LOG_UINT16, extVbatMV, &extBatteryVoltageMV)
LOG_ADD(LOG_FLOAT, extCurr, &extBatteryCurrent)
LOG_ADD(LOG_FLOAT, chargeCurrent, &pmSyslinkInfo.chargeCurrent)
LOG_ADD(LOG_INT8, state, &pmState)
LOG_ADD(LOG_UINT8, batteryLevel, &batteryLevel)
LOG_GROUP_STOP(pm)
