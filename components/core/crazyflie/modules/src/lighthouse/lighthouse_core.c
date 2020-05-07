/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
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
 *
 * lighthouse_core.c - central part of the lighthouse positioning system
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "system.h"
#include "log.h"
#include "param.h"
#include "statsCnt.h"

#define DEBUG_MODULE "LH"
#include "debug_cf.h"
#include "uart1.h"

#include "pulse_processor.h"
#include "pulse_processor_v1.h"
#include "pulse_processor_v2.h"

#include "lighthouse_deck_flasher.h"
#include "lighthouse_position_est.h"
#include "lighthouse_core.h"

#include "test_support.h"


static pulseProcessorResult_t angles;
static lighthouseUartFrame_t frame;
static lighthouseBsIdentificationData_t bsIdentificationData;

// Stats
static bool uartSynchronized = false;

#define ONE_SECOND 1000
#define HALF_SECOND 500
static STATS_CNT_RATE_DEFINE(serialFrameRate, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(frameRate, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(cycleRate, ONE_SECOND);

static STATS_CNT_RATE_DEFINE(bs0Rate, HALF_SECOND);
static STATS_CNT_RATE_DEFINE(bs1Rate, HALF_SECOND);
static statsCntRateLogger_t* bsRates[2] = {&bs0Rate, &bs1Rate};

static uint16_t pulseWidth[PULSE_PROCESSOR_N_SENSORS];
static pulseProcessor_t ppState = {};

pulseProcessorProcessPulse_t pulseProcessorProcessPulse = (void*)0;

#define UART_FRAME_LENGTH 12

static bool getUartFrameRaw(lighthouseUartFrame_t *frame) {
  static char data[UART_FRAME_LENGTH];
  int syncCounter = 0;

  for(int i = 0; i < UART_FRAME_LENGTH; i++) {
    uart1Getchar(&data[i]);
    if ((unsigned char)data[i] == 0xff) {
      syncCounter += 1;
    }
  }

  memset(frame, 0, sizeof(*frame));

  frame->isSyncFrame = (syncCounter == UART_FRAME_LENGTH);

  frame->data.sensor = data[0] & 0x03;
  frame->data.channelFound = (data[0] & 0x80) == 0;
  frame->data.channel = (data[0] >> 3) & 0x0f;
  frame->data.slowbit = (data[0] >> 2) & 0x01;
  memcpy(&frame->data.width, &data[1], 2);
  memcpy(&frame->data.offset, &data[3], 3);
  memcpy(&frame->data.beamData, &data[6], 3);
  memcpy(&frame->data.timestamp, &data[9], 3);

  // Offset is expressed in a 6 MHz clock, convert to the 24 MHz that is used for timestamps
  frame->data.offset *= 4;

  bool isPaddingZero = (((data[5] | data[8]) & 0xfe) == 0);
  bool isFrameValid = (isPaddingZero || frame->isSyncFrame);

  STATS_CNT_RATE_EVENT(&serialFrameRate);

  return isFrameValid;
}

TESTABLE_STATIC bool getUartFrame(lighthouseUartFrame_t *frame) {
  do {
    bool isUartFrameValid = getUartFrameRaw(frame);
    if (! isUartFrameValid) {
      return false;
    }
  } while(frame->isSyncFrame);

  return true;
}

TESTABLE_STATIC void waitForUartSynchFrame() {
  char c;
  int syncCounter = 0;
  bool synchronized = false;

  while (!synchronized) {
    uart1Getchar(&c);
    if ((unsigned char)c == 0xff) {
      syncCounter += 1;
    } else {
      syncCounter = 0;
    }
    synchronized = (syncCounter == UART_FRAME_LENGTH);
  }
}


// Method used to estimate position
// 0 = Position calculated outside the estimator using intersection point of beams.
//     Yaw error calculated outside the estimator. Position and yaw error is pushed to the
//     estimator as pre-calculated.
// 1 = Sweep angles pushed into the estimator. Yaw error calculated outside the estimator
//     and pushed to the estimator as a pre-calculated value.
static uint8_t estimationMethod = 1;


static void usePulseResultCrossingBeams(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int basestation, int axis) {
  if (basestation == 1) {
    STATS_CNT_RATE_EVENT(&cycleRate);

    lighthousePositionEstimatePoseCrossingBeams(angles, 1);

    pulseProcessorClear(angles, 0);
    pulseProcessorClear(angles, 1);
  }
}

static void usePulseResultSweeps(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int basestation, int axis) {
  STATS_CNT_RATE_EVENT(&cycleRate);

  lighthousePositionEstimatePoseSweeps(angles, basestation);

  pulseProcessorClear(angles, basestation);
}

static void usePulseResult(pulseProcessor_t *appState, pulseProcessorResult_t* angles, int basestation, int axis) {
  if (axis == sweepDirection_y) {
    pulseProcessorApplyCalibration(appState, angles, basestation);

    switch(estimationMethod) {
      case 0:
        usePulseResultCrossingBeams(appState, angles, basestation, axis);
        break;
      case 1:
        usePulseResultSweeps(appState, angles, basestation, axis);
        break;
      default:
        break;
    }
  }
}

/**
 * @brief Identify the type of base stations used in the system.
 * There is not member of the UART frame data we can use to directly identify the type, but we can use statistical methods.
 * The beamWord will vary for V2 base stations, while it will have the value 0x1ffff fairly offen for V1 base stations.
 *
 * @param frame
 * @param state
 * @return TESTABLE_STATIC identifyBaseStationType
 */
TESTABLE_STATIC lighthouseBaseStationType_t identifyBaseStationType(const lighthouseUartFrame_t* frame, lighthouseBsIdentificationData_t* state) {
    const uint_fast32_t v1Indicator = 0x1ffff;
    const int requiredIndicatorsForV1 = 6;
    const int requiredSamplesForV2 = 20;
    state->sampleCount++;
    if (frame->data.beamData == v1Indicator) {
        state->hitCount++;
    }

    if (state->hitCount >= requiredIndicatorsForV1) {
        return lighthouseBsTypeV1;
    }

    if (state->sampleCount >= requiredSamplesForV2) {
        return lighthouseBsTypeV2;
    }

    return lighthouseBsTypeUnknown;
}

static pulseProcessorProcessPulse_t identifySystem(const lighthouseUartFrame_t* frame, lighthouseBsIdentificationData_t* bsIdentificationData) {
  pulseProcessorProcessPulse_t result = (void*)0;

  switch (identifyBaseStationType(frame, bsIdentificationData)) {
    case lighthouseBsTypeV1:
      DEBUG_PRINT("Locking to V1 system\n");
      result = pulseProcessorV1ProcessPulse;
      break;
    case lighthouseBsTypeV2:
      DEBUG_PRINT("Locking to V2 system\n");
      result = pulseProcessorV2ProcessPulse;
      break;
    default:
      // Nothing here
      break;
  }

  return result;
}

static void processFrame(pulseProcessor_t *appState, pulseProcessorResult_t* angles, const lighthouseUartFrame_t* frame) {
    int basestation;
    int axis;

    pulseWidth[frame->data.sensor] = frame->data.width;

    if (pulseProcessorProcessPulse(&ppState, &frame->data, angles, &basestation, &axis)) {
        STATS_CNT_RATE_EVENT(bsRates[basestation]);
        usePulseResult(appState, angles, basestation, axis);
    }
}

void lighthouseCoreTask(void *param) {
  bool isUartFrameValid = false;

  uart1Init(230400);
  lightHousePositionGeometryDataUpdated();
  systemWaitStart();

  lighthouseDeckFlasherCheckVersionAndBoot();

  memset(&bsIdentificationData, 0, sizeof(bsIdentificationData));

  while(1) {
    memset(pulseWidth, 0, sizeof(pulseWidth[0])*PULSE_PROCESSOR_N_SENSORS);
    waitForUartSynchFrame();
    uartSynchronized = true;

    isUartFrameValid = getUartFrame(&frame);
    while(isUartFrameValid) {
      STATS_CNT_RATE_EVENT(&frameRate);

      if (pulseProcessorProcessPulse) {
        processFrame(&ppState, &angles, &frame);
      } else {
        pulseProcessorProcessPulse = identifySystem(&frame, &bsIdentificationData);
      }

      isUartFrameValid = getUartFrame(&frame);
    }

    uartSynchronized = false;
  }
}

LOG_GROUP_START(lighthouse)
LOG_ADD(LOG_FLOAT, rawAngle0x, &angles.sensorMeasurements[0].baseStatonMeasurements[0].angles[0])
LOG_ADD(LOG_FLOAT, rawAngle0y, &angles.sensorMeasurements[0].baseStatonMeasurements[0].angles[1])
LOG_ADD(LOG_FLOAT, rawAngle1x, &angles.sensorMeasurements[0].baseStatonMeasurements[1].angles[0])
LOG_ADD(LOG_FLOAT, rawAngle1y, &angles.sensorMeasurements[0].baseStatonMeasurements[1].angles[1])
LOG_ADD(LOG_FLOAT, angle0x, &angles.sensorMeasurements[0].baseStatonMeasurements[0].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle0y, &angles.sensorMeasurements[0].baseStatonMeasurements[0].correctedAngles[1])
LOG_ADD(LOG_FLOAT, angle1x, &angles.sensorMeasurements[0].baseStatonMeasurements[1].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle1y, &angles.sensorMeasurements[0].baseStatonMeasurements[1].correctedAngles[1])

LOG_ADD(LOG_FLOAT, angle0x_1, &angles.sensorMeasurements[1].baseStatonMeasurements[0].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle0y_1, &angles.sensorMeasurements[1].baseStatonMeasurements[0].correctedAngles[1])
LOG_ADD(LOG_FLOAT, angle1x_1, &angles.sensorMeasurements[1].baseStatonMeasurements[1].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle1y_1, &angles.sensorMeasurements[1].baseStatonMeasurements[1].correctedAngles[1])

LOG_ADD(LOG_FLOAT, angle0x_2, &angles.sensorMeasurements[2].baseStatonMeasurements[0].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle0y_2, &angles.sensorMeasurements[2].baseStatonMeasurements[0].correctedAngles[1])
LOG_ADD(LOG_FLOAT, angle1x_2, &angles.sensorMeasurements[2].baseStatonMeasurements[1].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle1y_2, &angles.sensorMeasurements[2].baseStatonMeasurements[1].correctedAngles[1])

LOG_ADD(LOG_FLOAT, angle0x_3, &angles.sensorMeasurements[3].baseStatonMeasurements[0].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle0y_3, &angles.sensorMeasurements[3].baseStatonMeasurements[0].correctedAngles[1])
LOG_ADD(LOG_FLOAT, angle1x_3, &angles.sensorMeasurements[3].baseStatonMeasurements[1].correctedAngles[0])
LOG_ADD(LOG_FLOAT, angle1y_3, &angles.sensorMeasurements[3].baseStatonMeasurements[1].correctedAngles[1])

STATS_CNT_RATE_LOG_ADD(serRt, &serialFrameRate)
STATS_CNT_RATE_LOG_ADD(frmRt, &frameRate)
STATS_CNT_RATE_LOG_ADD(cycleRt, &cycleRate)

STATS_CNT_RATE_LOG_ADD(bs0Rt, &bs0Rate)
STATS_CNT_RATE_LOG_ADD(bs1Rt, &bs1Rate)

LOG_ADD(LOG_UINT16, width0, &pulseWidth[0])
LOG_ADD(LOG_UINT16, width1, &pulseWidth[1])
LOG_ADD(LOG_UINT16, width2, &pulseWidth[2])
LOG_ADD(LOG_UINT16, width3, &pulseWidth[3])

LOG_ADD(LOG_UINT8, comSync, &uartSynchronized)
LOG_GROUP_STOP(lighthouse)

PARAM_GROUP_START(lighthouse)
PARAM_ADD(PARAM_UINT8, method, &estimationMethod)
PARAM_GROUP_STOP(lighthouse)
