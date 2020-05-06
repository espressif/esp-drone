#pragma once

#include <stdbool.h>

// Naive 3d vector type.
#define vec3d_size 3
typedef float vec3d[vec3d_size];

typedef struct baseStationGeometry_s {
  float origin[3];
  float mat[3][3];
} __attribute__((packed)) baseStationGeometry_t;

bool lighthouseGeometryGetPosition(baseStationGeometry_t baseStations[2], float angles[4], vec3d position, float *position_delta);
