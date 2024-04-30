/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2012 BitCraze AB
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
 * wifilink.c: ESP32 Wi-Fi implementation of the CRTP link
 */

#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "wifilink.h"
#include "wifi_esp32.h"
#include "crtp.h"
#include "configblock.h"
#include "ledseq.h"
#include "pm_esplane.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "queuemonitor.h"
#include "semphr.h"
#include "stm32_legacy.h"

#define DEBUG_MODULE "WIFILINK"
#include "debug_cf.h"
#include "static_mem.h"

#define WIFI_ACTIVITY_TIMEOUT_MS (1000)

static bool isInit = false;
static xQueueHandle crtpPacketDelivery;
STATIC_MEM_QUEUE_ALLOC(crtpPacketDelivery, 16, sizeof(CRTPPacket));
static UDPPacket wifiIn;
static uint32_t lastPacketTick;

static int wifilinkSendPacket(CRTPPacket *p);
static int wifilinkSetEnable(bool enable);
static int wifilinkReceiveCRTPPacket(CRTPPacket *p);

STATIC_MEM_TASK_ALLOC(wifilinkTask, WIFILINK_TASK_STACKSIZE);

static bool wifilinkIsConnected(void)
{
    return (xTaskGetTickCount() - lastPacketTick) < M2T(WIFI_ACTIVITY_TIMEOUT_MS);
}

static struct crtpLinkOperations wifilinkOp = {
    .setEnable         = wifilinkSetEnable,
    .sendPacket        = wifilinkSendPacket,
    .receivePacket     = wifilinkReceiveCRTPPacket,
    .isConnected       = wifilinkIsConnected,
};

#ifdef CONFIG_ENABLE_LEGACY_APP
static bool detectOldVersionApp(UDPPacket *in)
{

    if ((in->data)[0] != 0x00 && in->size == 11) { //12-1
        if ((in->data)[0] == 0x80 && (in->data)[9] == 0x00 && (in->data)[10] == 0x00 && (in->data)[11] == 0x00) {
            return true;
        }
    }

    return false;
}
#endif

static bool detectEspNow(UDPPacket *in)
{
    if (in->size == 7 && (in->data)[0] == 'n' && (in->data)[1] == 'o' && (in->data)[2] == 'w') {
        return true;
    }
    return false;
}

static void wifilinkTask(void *param)
{
    CRTPPacket p = {0};
    while (1) {
        /* command step - receive  03 Fetch a wifi packet off the queue */
        wifiGetDataBlocking(&wifiIn);
        lastPacketTick = xTaskGetTickCount();
        uint16_t sendWaitMs = 0;
#ifdef CONFIG_ENABLE_LEGACY_APP
        float rch, pch, ych;
        uint16_t tch;
        if (detectOldVersionApp(&wifiIn)) {
            rch  = (1.0) * (float)(((((uint16_t)wifiIn.data[1] << 8) + (uint16_t)wifiIn.data[2]) - 296) * 15.0 / 150.0); //-15~+15
            pch  = (-1.0) * (float)(((((uint16_t)wifiIn.data[3] << 8) + (uint16_t)wifiIn.data[4]) - 296) * 15.0 / 150.0); //-15~+15
            tch  = (((uint16_t)wifiIn.data[5] << 8) + (uint16_t)wifiIn.data[6]) * 59000.0 / 600.0;
            ych  = (float)(((((uint16_t)wifiIn.data[7] << 8) + (uint16_t)wifiIn.data[8]) - 296) * 15.0 / 150.0); //-15~+15
            p.size = wifiIn.size - 1;
            p.header = CRTP_HEADER(CRTP_PORT_SETPOINT, 0x00); //head redefine

            memcpy(&p.data[0], &rch, 4);
            memcpy(&p.data[4], &pch, 4);
            memcpy(&p.data[8], &ych, 4);
            memcpy(&p.data[12], &tch, 2);
        } else
#endif
        if (detectEspNow(&wifiIn)) {
            float rch, pch, ych;
            uint16_t tch;
            rch  = (float)((int8_t)wifiIn.data[6] * 15.0 / 128);; //-15~+15
            pch  = (float)((int8_t)wifiIn.data[5] * 15.0 / -128);; //-15~+15
            if ((int8_t)wifiIn.data[4] < 0) {
                tch  = 0;
            } else {
                tch  = (int8_t)wifiIn.data[4] * 59000.0 / 128;
            }
            ych  = (float)((int8_t)wifiIn.data[3] * 15.0 / 128); //-15~+15
            p.size = wifiIn.size - 1;
            p.header = CRTP_HEADER(CRTP_PORT_SETPOINT, 0x00); //head redefine

            //printf("rch: %.2f, pch: %.2f, tch: %d, ych: %.2f\n", rch, pch, tch, ych);
            memcpy(&p.data[0], &rch, 4);
            memcpy(&p.data[4], &pch, 4);
            memcpy(&p.data[8], &ych, 4);
            memcpy(&p.data[12], &tch, 2);
        } else
        {
            /* command step - receive  04 copy CRTP part from packet, the size not contain head */
            p.size = wifiIn.size - 1;
            ASSERT(p.size <= CRTP_MAX_DATA_SIZE);
            memcpy(&p.raw, wifiIn.data, wifiIn.size);
            sendWaitMs = 10;
        }

        /* command step - receive 05 send to crtpPacketDelivery queue */
        xQueueSend(crtpPacketDelivery, &p, M2T(sendWaitMs));
    }

}

static int wifilinkReceiveCRTPPacket(CRTPPacket *p)
{
    if (xQueueReceive(crtpPacketDelivery, p, M2T(100)) == pdTRUE) {
        ledseqRun(&seq_linkUp);
        return 0;
    }

    return -1;
}

static int wifilinkSendPacket(CRTPPacket *p)
{
    ASSERT(p->size <= CRTP_MAX_DATA_SIZE);
    ledseqRun(&seq_linkDown);
    return wifiSendData(p->size + 1, p->raw);
}

static int wifilinkSetEnable(bool enable)
{
    return 0;
}

/*
 * Public functions
 */

void wifilinkInit()
{
    if (isInit) {
        return;
    }

    crtpPacketDelivery = STATIC_MEM_QUEUE_CREATE(crtpPacketDelivery);
    DEBUG_QUEUE_MONITOR_REGISTER(crtpPacketDelivery);

    STATIC_MEM_TASK_CREATE(wifilinkTask, wifilinkTask, WIFILINK_TASK_NAME,NULL, WIFILINK_TASK_PRI);

    isInit = true;
}

bool wifilinkTest()
{
    return isInit;
}

struct crtpLinkOperations *wifilinkGetLink()
{
    return &wifilinkOp;
}
