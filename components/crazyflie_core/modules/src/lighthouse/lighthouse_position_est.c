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
 * lighthouse_position_est.c - position estimaton for the lighthouse system
 */

#include "stabilizer_types.h"
#include "estimator.h"
#include "estimator_kalman.h"

#include "log.h"
#include "param.h"
#include "statsCnt.h"

#include "lighthouse_position_est.h"

baseStationGeometry_t lighthouseBaseStationsGeometry[2]  = {
{.origin = {-1.958483,  0.542299,  3.152727, }, .mat = {{0.79721498, -0.004274, 0.60368103, }, {0.0, 0.99997503, 0.00708, }, {-0.60369599, -0.005645, 0.79719502, }, }},
{.origin = {1.062398, -2.563488,  3.112367, }, .mat = {{0.018067, -0.999336, 0.031647, }, {0.76125097, 0.034269, 0.64755201, }, {-0.648206, 0.012392, 0.76136398, }, }},
};


#define ONE_SECOND 1000
#define HALF_SECOND 500
static STATS_CNT_RATE_DEFINE(positionRate, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(estBs0Rate, HALF_SECOND);
static STATS_CNT_RATE_DEFINE(estBs1Rate, HALF_SECOND);
static statsCntRateLogger_t* bsEstRates[2] = {&estBs0Rate, &estBs1Rate};

baseStationEulerAngles_t lighthouseBaseStationAngles[2];
static mat3d baseStationInvertedRotationMatrixes[2];

static void invertRotationMatrix(mat3d rot, mat3d inverted);

void lightHousePositionGeometryDataUpdated() {
  lighthouseGeometryCalculateAnglesFromRotationMatrix(&lighthouseBaseStationsGeometry[0], &lighthouseBaseStationAngles[0]);
  lighthouseGeometryCalculateAnglesFromRotationMatrix(&lighthouseBaseStationsGeometry[1], &lighthouseBaseStationAngles[1]);

  invertRotationMatrix(lighthouseBaseStationsGeometry[0].mat, baseStationInvertedRotationMatrixes[0]);
  invertRotationMatrix(lighthouseBaseStationsGeometry[1].mat, baseStationInvertedRotationMatrixes[1]);
}


static void invertRotationMatrix(mat3d rot, mat3d inverted) {
  // arm_mat_inverse_f32() alters the original matrix in the process, must make a copy to work from
  float bs_r_tmp[3][3];
  memcpy(bs_r_tmp, (float32_t *)rot, sizeof(bs_r_tmp));
  arm_matrix_instance_f32 basestation_rotation_matrix_tmp = {3, 3, (float32_t *)bs_r_tmp};

  arm_matrix_instance_f32 basestation_rotation_matrix_inv = {3, 3, (float32_t *)inverted};
  arm_mat_inverse_f32(&basestation_rotation_matrix_tmp, &basestation_rotation_matrix_inv);
}


// Sensor positions on the deck
#define SENSOR_POS_W (0.015f / 2.0f)
#define SENSOR_POS_L (0.030f / 2.0f)
static vec3d sensorDeckPositions[4] = {
    {-SENSOR_POS_L, SENSOR_POS_W, 0.0},
    {-SENSOR_POS_L, -SENSOR_POS_W, 0.0},
    {SENSOR_POS_L, SENSOR_POS_W, 0.0},
    {SENSOR_POS_L, -SENSOR_POS_W, 0.0},
};


static positionMeasurement_t ext_pos;
static float sweepStd = 0.0004;

static vec3d position;
static float deltaLog;

static void estimatePositionCrossingBeams(pulseProcessorResult_t* angles, int baseStation) {
  memset(&ext_pos, 0, sizeof(ext_pos));
  int sensorsUsed = 0;
  float delta;

  // Average over all sensors with valid data
  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
      pulseProcessorBaseStationMeasuremnt_t* bs0Measurement = &angles->sensorMeasurements[sensor].baseStatonMeasurements[0];
      pulseProcessorBaseStationMeasuremnt_t* bs1Measurement = &angles->sensorMeasurements[sensor].baseStatonMeasurements[1];

      if (bs0Measurement->validCount == PULSE_PROCESSOR_N_SWEEPS && bs1Measurement->validCount == PULSE_PROCESSOR_N_SWEEPS) {
        lighthouseGeometryGetPositionFromRayIntersection(lighthouseBaseStationsGeometry, bs0Measurement->correctedAngles, bs1Measurement->correctedAngles, position, &delta);

        deltaLog = delta;

        ext_pos.x += position[0];
        ext_pos.y += position[1];
        ext_pos.z += position[2];
        sensorsUsed++;

        STATS_CNT_RATE_EVENT(&positionRate);
      }
  }

  ext_pos.x /= sensorsUsed;
  ext_pos.y /= sensorsUsed;
  ext_pos.z /= sensorsUsed;

  // Make sure we feed sane data into the estimator
  if (isfinite(ext_pos.pos[0]) && isfinite(ext_pos.pos[1]) && isfinite(ext_pos.pos[2])) {
    ext_pos.stdDev = 0.01;
    estimatorEnqueuePosition(&ext_pos);
  }
}

static void estimatePositionSweeps(pulseProcessorResult_t* angles, int baseStation) {
  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    pulseProcessorBaseStationMeasuremnt_t* bsMeasurement = &angles->sensorMeasurements[sensor].baseStatonMeasurements[baseStation];
    if (bsMeasurement->validCount == PULSE_PROCESSOR_N_SWEEPS) {
      sweepAngleMeasurement_t sweepAngles;
      sweepAngles.angleX = bsMeasurement->correctedAngles[0];
      sweepAngles.angleY = bsMeasurement->correctedAngles[1];

      if (sweepAngles.angleX != 0 && sweepAngles.angleY != 0) {
        sweepAngles.stdDevX = sweepStd;
        sweepAngles.stdDevY = sweepStd;

        sweepAngles.sensorPos = &sensorDeckPositions[sensor];

        sweepAngles.baseStationPos = &lighthouseBaseStationsGeometry[baseStation].origin;
        sweepAngles.baseStationRot = &lighthouseBaseStationsGeometry[baseStation].mat;
        sweepAngles.baseStationRotInv = &baseStationInvertedRotationMatrixes[baseStation];

        estimatorEnqueueSweepAngles(&sweepAngles);
        STATS_CNT_RATE_EVENT(bsEstRates[baseStation]);
      }
    }
  }
}

static bool estimateYawDeltaOneBaseStation(const int bs, const pulseProcessorResult_t* angles, baseStationGeometry_t baseStationGeometries[], const float cfPos[3], const float n[3], const arm_matrix_instance_f32 *RR, float *yawDelta) {
  baseStationGeometry_t* baseStationGeometry = &baseStationGeometries[bs];

  vec3d baseStationPos;
  lighthouseGeometryGetBaseStationPosition(baseStationGeometry, baseStationPos);

  vec3d rays[PULSE_PROCESSOR_N_SENSORS];
  for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    const pulseProcessorBaseStationMeasuremnt_t* bsMeasurement = &angles->sensorMeasurements[sensor].baseStatonMeasurements[bs];
    lighthouseGeometryGetRay(baseStationGeometry, bsMeasurement->correctedAngles[0], bsMeasurement->correctedAngles[1], rays[sensor]);
  }

  // Intersection points of rays and the deck
  vec3d intersectionPoints[PULSE_PROCESSOR_N_SENSORS];
  for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    bool exists = lighthouseGeometryIntersectionPlaneVector(baseStationPos, rays[sensor], cfPos, n, intersectionPoints[sensor]);
    if (! exists) {
      return false;
    }
  }

  // Calculate positions of sensors. Rotate relative postiions using the rotation matrix and add current position
  vec3d sensorPoints[PULSE_PROCESSOR_N_SENSORS];
  for (int sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
    lighthouseGeometryGetSensorPosition(cfPos, RR, sensorDeckPositions[sensor], sensorPoints[sensor]);
  }

  // Calculate diagonals (sensors 0 - 3 and 1 - 2) for intersection and sensor points
  vec3d ipv1 = {intersectionPoints[3][0] - intersectionPoints[0][0], intersectionPoints[3][1] - intersectionPoints[0][1], intersectionPoints[3][2] - intersectionPoints[0][2]};
  vec3d ipv2 = {intersectionPoints[2][0] - intersectionPoints[1][0], intersectionPoints[2][1] - intersectionPoints[1][1], intersectionPoints[2][2] - intersectionPoints[1][2]};
  vec3d spv1 = {sensorPoints[3][0] - sensorPoints[0][0], sensorPoints[3][1] - sensorPoints[0][1], sensorPoints[3][2] - sensorPoints[0][2]};
  vec3d spv2 = {sensorPoints[2][0] - sensorPoints[1][0], sensorPoints[2][1] - sensorPoints[1][1], sensorPoints[2][2] - sensorPoints[1][2]};

  // Calculate yaw delta for the two diagonals and average
  float yawDelta1, yawDelta2;
  if (lighthouseGeometryYawDelta(ipv1, spv1, n, &yawDelta1) && lighthouseGeometryYawDelta(ipv2, spv2, n, &yawDelta2)) {
    *yawDelta = (yawDelta1 + yawDelta2) / 2.0f;
    return true;
   } else {
    *yawDelta = 0.0f;
    return false;
  }
}

static void estimateYaw(pulseProcessorResult_t* angles, int baseStation) {
  // TODO Most of these calculations should be moved into the estimator instead. It is a
  // bit dirty to get the state from the kalman filer here and calculate the yaw error outside
  // the estimator, but it will do for now.

  // Get data from the current estimated state
  point_t cfPosP;
  estimatorKalmanGetEstimatedPos(&cfPosP);
  vec3d cfPos = {cfPosP.x, cfPosP.y, cfPosP.z};

  // Rotation matrix
  float R[3][3];
  estimatorKalmanGetEstimatedRot((float*)R);
  arm_matrix_instance_f32 RR = {3, 3, (float*)R};

  // Normal to the deck: (0, 0, 1), rotated using the rotation matrix
  const vec3d n = {R[0][2], R[1][2], R[2][2]};

  // Calculate yaw delta using only one base station for now
  float yawDelta;
  if (estimateYawDeltaOneBaseStation(baseStation, angles, lighthouseBaseStationsGeometry, cfPos, n, &RR, &yawDelta)) {
    yawErrorMeasurement_t yawDeltaMeasurement = {.yawError = yawDelta, .stdDev = 0.01};
    estimatorEnqueueYawError(&yawDeltaMeasurement);
  }
}

void lighthousePositionEstimatePoseCrossingBeams(pulseProcessorResult_t* angles, int baseStation) {
  estimatePositionCrossingBeams(angles, baseStation);
  estimateYaw(angles, baseStation);
}

void lighthousePositionEstimatePoseSweeps(pulseProcessorResult_t* angles, int baseStation) {
  estimatePositionSweeps(angles, baseStation);
  estimateYaw(angles, baseStation);
}

LOG_GROUP_START(lighthouse)
STATS_CNT_RATE_LOG_ADD(posRt, &positionRate)
STATS_CNT_RATE_LOG_ADD(estBs0Rt, &estBs0Rate)
STATS_CNT_RATE_LOG_ADD(estBs1Rt, &estBs1Rate)

LOG_ADD(LOG_FLOAT, x, &position[0])
LOG_ADD(LOG_FLOAT, y, &position[1])
LOG_ADD(LOG_FLOAT, z, &position[2])

LOG_ADD(LOG_FLOAT, delta, &deltaLog)
LOG_GROUP_STOP(lighthouse)

PARAM_GROUP_START(lighthouse)
PARAM_ADD(PARAM_FLOAT, sweepStd, &sweepStd)
PARAM_GROUP_STOP(lighthouse)
