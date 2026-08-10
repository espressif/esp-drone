#pragma once

/*
 * Architecture-neutral facade for the bundled portable C DSP backend.
 *
 * The backend retains its historical xtensa_* ABI for compatibility, but it
 * does not contain Xtensa instructions and is also built for RISC-V targets.
 * Flight-control code must use the dsp_* names below so an optimized backend
 * can be selected later without leaking a CPU architecture into the API.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "xtensa_math.h"
#pragma GCC diagnostic pop

typedef xtensa_matrix_instance_f32 dsp_matrix_instance_f32;
typedef xtensa_status dsp_status_t;

#define DSP_MATH_SUCCESS XTENSA_MATH_SUCCESS

static inline dsp_status_t dsp_mat_trans_f32(const dsp_matrix_instance_f32 *source,
                                              dsp_matrix_instance_f32 *destination)
{
    return xtensa_mat_trans_f32(source, destination);
}

static inline dsp_status_t dsp_mat_inverse_f32(const dsp_matrix_instance_f32 *source,
                                                dsp_matrix_instance_f32 *destination)
{
    return xtensa_mat_inverse_f32(source, destination);
}

static inline dsp_status_t dsp_mat_mult_f32(const dsp_matrix_instance_f32 *sourceA,
                                             const dsp_matrix_instance_f32 *sourceB,
                                             dsp_matrix_instance_f32 *destination)
{
    return xtensa_mat_mult_f32(sourceA, sourceB, destination);
}

static inline dsp_status_t dsp_sqrt_f32(float input, float *output)
{
    return xtensa_sqrt_f32(input, output);
}

static inline float dsp_sin_f32(float input)
{
    return xtensa_sin_f32(input);
}

static inline float dsp_cos_f32(float input)
{
    return xtensa_cos_f32(input);
}

static inline void dsp_mean_f32(const float *source, uint32_t length, float *result)
{
    xtensa_mean_f32(source, length, result);
}
