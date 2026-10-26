/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2018 Bitcraze AB
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
 * Utilities to simplify unit testing
 *
 */

#pragma once

#include <stdint.h>

#include "dsp_math.h"

#include "cfassert.h"


#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)

#ifndef MIN
#define MIN(a, b) ((b) < (a) ? (b) : (a))
#endif
#ifndef MAX
#define MAX(a, b) ((b) > (a) ? (b) : (a))
#endif

// Matrix data must be aligned on 4 byte bundaries
static inline void assert_aligned_4_bytes(const dsp_matrix_instance_f32 *matrix)
{
    const uintptr_t address = (uintptr_t)matrix->pData;
    ASSERT((address & 0x3) == 0);
}

static inline void mat_trans(const dsp_matrix_instance_f32 *pSrc, dsp_matrix_instance_f32 *pDst)
{
    assert_aligned_4_bytes(pSrc);
    assert_aligned_4_bytes(pDst);

    ASSERT(DSP_MATH_SUCCESS == dsp_mat_trans_f32(pSrc, pDst));
}

static inline void mat_inv(const dsp_matrix_instance_f32 *pSrc, dsp_matrix_instance_f32 *pDst)
{
    assert_aligned_4_bytes(pSrc);
    assert_aligned_4_bytes(pDst);

    ASSERT(DSP_MATH_SUCCESS == dsp_mat_inverse_f32(pSrc, pDst));
}

static inline void mat_mult(const dsp_matrix_instance_f32 *pSrcA, const dsp_matrix_instance_f32 *pSrcB, dsp_matrix_instance_f32 *pDst)
{
    assert_aligned_4_bytes(pSrcA);
    assert_aligned_4_bytes(pSrcB);
    assert_aligned_4_bytes(pDst);

    ASSERT(DSP_MATH_SUCCESS == dsp_mat_mult_f32(pSrcA, pSrcB, pDst));
}

static inline float cf_sqrt(float in)
{
    float output = 0;
    const dsp_status_t result = dsp_sqrt_f32(in, &output);
    ASSERT(DSP_MATH_SUCCESS == result);
    return output;
}

static inline float limPos(float in)
{
    if (in < 0.0f) {
        return 0.0f;
    }

    return in;
}

static inline float clip1(float a)
{
    if (a < -1.0f) {
        return -1.0f;
    }

    if (a > 1.0f) {
        return 1.0f;
    }

    return a;
}
