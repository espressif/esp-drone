#pragma once

#include "ootx_decoder.h"

typedef struct ligthouseCalibration_s {
  bool valid;

  struct {
    float phase;
    float tilt;
    float curve;
    float gibmag;
    float gibphase;
  } axis[2];
} lighthouseCalibration_t;

/**
 * @brief Initialize calibration structure from basestation ootx frame
 * 
 * @param calib Calibration  object to initialize, calib->valid will be set to true
 * @param frame ootx frame received from the basestation
 */
void lighthouseCalibrationInitFromFrame(lighthouseCalibration_t *calib, struct ootxDataFrame_s *frame);

/**
 * @brief Apply basestation calibration to the two received angles
 * 
 * Only applies calibration if valid is true.
 * 
 * @param calib Calibration object to use
 * @param rawAngles i/j raw angles measured
 * @param correctedAngles i/j corrected angles after applying calibration
 */
void lighthouseCalibrationApply(lighthouseCalibration_t* calib, float rawAngles[2], float correctedAngles[2]);