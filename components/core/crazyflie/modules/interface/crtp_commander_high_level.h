/*
 *    ______
 *   / ____/________ _____  __  ________      ______ __________ ___
 *  / /   / ___/ __ `/_  / / / / / ___/ | /| / / __ `/ ___/ __ `__ \
 * / /___/ /  / /_/ / / /_/ /_/ (__  )| |/ |/ / /_/ / /  / / / / / /
 * \____/_/   \__,_/ /___/\__, /____/ |__/|__/\__,_/_/  /_/ /_/ /_/
 *                       /____/
 *
 * Crazyswarm advanced control firmware for Crazyflie
 *

The MIT License (MIT)

Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
Header file for high-level commander that computes smooth setpoints based on high-level inputs.
*/

#ifndef CRTP_COMMANDER_HIGH_LEVEL_H_
#define CRTP_COMMANDER_HIGH_LEVEL_H_

#include <stdbool.h>
#include <stdint.h>

#include "math3d.h"

#include "stabilizer_types.h"

// allocate memory to store trajectories
// 4k allows us to store 31 poly4d pieces
// other (compressed) formats might be added in the future
#define TRAJECTORY_MEMORY_SIZE 4096
extern uint8_t trajectories_memory[TRAJECTORY_MEMORY_SIZE];

#define NUM_TRAJECTORY_DEFINITIONS 10

/* Public functions */
void crtpCommanderHighLevelInit(void);

// Retrieves the current setpoint
void crtpCommanderHighLevelGetSetpoint(setpoint_t* setpoint, const state_t *state);

// When flying sequences of high-level commands, the high-level commander uses
// its own history of commands to determine the initial conditions of the next
// trajectory it plans. However, when switching from a low-level streaming
// setpoint mode to high-level, any past command history is invalid because of
// the intervening low-level commands. Therefore, we must tell the high-level
// commander what initial conditions to use for trajectory planning.
void crtpCommanderHighLevelTellState(const state_t *state);

// Send the trajectory planner to idle state, where it has no plan. Used when
// switching from high-level to low-level commands, or for emergencies.
void crtpCommanderHighLevelStop();

// True if we have landed or emergency-stopped.
bool crtpCommanderHighLevelIsStopped();

#endif /* CRTP_COMMANDER_HIGH_LEVEL_H_ */
