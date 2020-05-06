/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
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
 */

/*
 * ledseq.h - LED sequence handler
 */

#ifndef __LEDSEQ_H__
#define __LEDSEQ_H__

/* A LED sequence is made of a list of actions. Each action contains the new
 * state of the LED and either a time to wait before executing the next action
 * or a command LOOP or STOP.
 *
 * The sequences are stored in a list by priority order ([0] is the highest
 * priority). The list ordered by priority is defined at the beginning of
 * ledseq.c
 *
 * Each sequence effects only one LED. For each LED only the runnable sequence
 * with the highest priority is run.
 */

#include <stdint.h>
#include <stdbool.h>
#include <led.h>

#define LEDSEQ_CHARGE_CYCLE_TIME_500MA  1000
#define LEDSEQ_CHARGE_CYCLE_TIME_MAX    500
//Led sequence action
#define LEDSEQ_WAITMS(X) (X)
#define LEDSEQ_STOP      -1
#define LEDSEQ_LOOP      -2
typedef struct {
  bool value;
  int action;
} ledseq_t;

//Public API
void ledseqInit(void);
bool ledseqTest(void);

void ledseqEnable(bool enable);
void ledseqRun(led_t led, const ledseq_t * sequence);
void ledseqStop(led_t led, const ledseq_t * sequence);
void ledseqSetTimes(ledseq_t *sequence, int32_t onTime, int32_t offTime);

//Existing led sequences
extern const ledseq_t seq_armed[];
extern const ledseq_t seq_calibrated[];
extern const ledseq_t seq_alive[];
extern const ledseq_t seq_lowbat[];
extern const ledseq_t seq_linkup[];
extern const ledseq_t seq_altHold[];
extern const ledseq_t seq_charged[];
extern ledseq_t seq_charging[];
extern ledseq_t seq_chargingMax[];
extern const ledseq_t seq_bootloader[];
extern const ledseq_t seq_testPassed[];

#endif
