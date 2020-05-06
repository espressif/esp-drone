#ifndef clockCorrectionEngine_h
#define clockCorrectionEngine_h

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  double clockCorrection;
  unsigned int clockCorrectionBucket;
} clockCorrectionStorage_t;

double clockCorrectionEngineGet(const clockCorrectionStorage_t* storage);
double clockCorrectionEngineCalculate(const uint64_t new_t_in_cl_reference, const uint64_t old_t_in_cl_reference, const uint64_t new_t_in_cl_x, const uint64_t old_t_in_cl_x, const uint64_t mask);
bool clockCorrectionEngineUpdate(clockCorrectionStorage_t* storage, const double clockCorrectionCandidate);

#endif /* clockCorrectionEngine_h */
