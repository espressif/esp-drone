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
 * ledseq.c - LED sequence handler
 */

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

#include "ledseq.h"
#include "led.h"
#include "stm32_legacy.h"

#ifdef CALIBRATED_LED_MORSE
  #define DOT 100
  #define DASH (3 * DOT)
  #define GAP DOT
  #define LETTER_GAP (3 * DOT)
  #define WORD_GAP (7 * DOT)
#endif // #ifdef CALIBRATED_LED_MORSE

/* Led sequence priority */
static ledseq_t const * sequences[] = {
  seq_testPassed,
  seq_lowbat,
  seq_charged,
  seq_charging,
  seq_chargingMax,
  seq_bootloader,
  seq_armed,
  seq_calibrated,
  seq_alive,
  seq_linkup,
};

/* Led sequences */
const ledseq_t seq_lowbat[] = {
  { true, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

const ledseq_t seq_armed[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(250)},
  {    0, LEDSEQ_LOOP},
};

const ledseq_t seq_calibrated[] = {
#ifndef CALIBRATED_LED_MORSE
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(450)},
  {    0, LEDSEQ_LOOP},
#else
  { true, LEDSEQ_WAITMS(DASH)},
  {false, LEDSEQ_WAITMS(GAP)},
  { true, LEDSEQ_WAITMS(DOT)},
  {false, LEDSEQ_WAITMS(GAP)},
  { true, LEDSEQ_WAITMS(DASH)},
  {false, LEDSEQ_WAITMS(GAP)},
  { true, LEDSEQ_WAITMS(DOT)},
  {false, LEDSEQ_WAITMS(LETTER_GAP)},
  { true, LEDSEQ_WAITMS(DOT)},
  {false, LEDSEQ_WAITMS(GAP)},
  { true, LEDSEQ_WAITMS(DOT)},
  {false, LEDSEQ_WAITMS(GAP)},
  { true, LEDSEQ_WAITMS(DASH)},
  {false, LEDSEQ_WAITMS(GAP)},
  { true, LEDSEQ_WAITMS(DOT)},
  {false, LEDSEQ_WAITMS(WORD_GAP)},
  {    0, LEDSEQ_LOOP},
#endif // ifndef CALIBRATED_LED_MORSE
};

const ledseq_t seq_alive[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(1950)},
  {    0, LEDSEQ_LOOP},
};


//TODO: Change, right now is called so fast it looks like seq_lowbat
const ledseq_t seq_altHold[] = {
  { true, LEDSEQ_WAITMS(1)},
  {false, LEDSEQ_WAITMS(50)},
  {    0, LEDSEQ_STOP},
};

const ledseq_t seq_linkup[] = {
  { true, LEDSEQ_WAITMS(1)},
  {false, LEDSEQ_WAITMS(0)},
  {    0, LEDSEQ_STOP},
};


const ledseq_t seq_charged[] = {
  { true, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_charging[] = {
  { true, LEDSEQ_WAITMS(200)},
  {false, LEDSEQ_WAITMS(800)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_chargingMax[] = {
  { true, LEDSEQ_WAITMS(100)},
  {false, LEDSEQ_WAITMS(400)},
  {    0, LEDSEQ_LOOP},
};

const ledseq_t seq_bootloader[] = {
  { true, LEDSEQ_WAITMS(500)},
  {false, LEDSEQ_WAITMS(500)},
  {    0, LEDSEQ_LOOP},
};

const ledseq_t seq_testPassed[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_STOP},
};

/* Led sequence handling machine implementation */
#define SEQ_NUM (sizeof(sequences)/sizeof(sequences[0]))

static void runLedseq(xTimerHandle xTimer);
static int getPrio(const ledseq_t *seq);
static void updateActive(led_t led);

//State of every sequence for every led: LEDSEQ_STOP if stopped or the current
//step
static int state[LED_NUM][SEQ_NUM];
//Active sequence for each led
static int activeSeq[LED_NUM];

static xTimerHandle timer[LED_NUM];
static StaticTimer_t timerBuffer[LED_NUM];

static xSemaphoreHandle ledseqSem;

static bool isInit = false;
static bool ledseqEnabled = false;

void ledseqInit()
{
  int i,j;

  if(isInit)
    return;

  ledInit();

  //Initialise the sequences state
  for(i=0; i<LED_NUM; i++) {
    activeSeq[i] = LEDSEQ_STOP;
    for(j=0; j<SEQ_NUM; j++)
      state[i][j] = LEDSEQ_STOP;
  }

  //Init the soft timers that runs the led sequences for each leds
  for(i=0; i<LED_NUM; i++) {
    timer[i] = xTimerCreateStatic("ledseqTimer", M2T(1000), pdFALSE, (void*)i, runLedseq, &timerBuffer[i]);
  }

  vSemaphoreCreateBinary(ledseqSem);

  isInit = true;
}

bool ledseqTest(void)
{
  bool status;

  status = isInit & ledTest();
  #ifdef TURN_OFF_LEDS
  ledseqEnable(false);
  ledSet(LED_BLUE, 0);
  #else
  ledseqEnable(true);
  #endif

  return status;
}

void ledseqEnable(bool enable)
{
  ledseqEnabled = enable;
}

void ledseqRun(led_t led, const ledseq_t *sequence)
{
  int prio = getPrio(sequence);

  if(prio<0) return;

  xSemaphoreTake(ledseqSem, portMAX_DELAY);
  state[led][prio] = 0;  //Reset the seq. to its first step
  updateActive(led);
  xSemaphoreGive(ledseqSem);

  //Run the first step if the new seq is the active sequence
  if(activeSeq[led] == prio)
    runLedseq(timer[led]);
}

void ledseqSetTimes(ledseq_t *sequence, int32_t onTime, int32_t offTime)
{
  sequence[0].action = onTime;
  sequence[1].action = offTime;
}

void ledseqStop(led_t led, const ledseq_t *sequence)
{
  int prio = getPrio(sequence);

  if(prio<0) return;

  xSemaphoreTake(ledseqSem, portMAX_DELAY);
  state[led][prio] = LEDSEQ_STOP;  //Stop the seq.
  updateActive(led);
  xSemaphoreGive(ledseqSem);

  //Run the next active sequence (if any...)
  runLedseq(timer[led]);
}

/* Center of the led sequence machine. This function is executed by the FreeRTOS
 * timer and runs the sequences
 */
static void runLedseq( xTimerHandle xTimer )
{
  led_t led = (led_t)pvTimerGetTimerID(xTimer);
  const ledseq_t *step;
  bool leave=false;

  if (!ledseqEnabled)
    return;

  while(!leave) {
    int prio = activeSeq[led];

    if (prio == LEDSEQ_STOP)
      return;

    step = &sequences[prio][state[led][prio]];

    state[led][prio]++;

    xSemaphoreTake(ledseqSem, portMAX_DELAY);
    switch(step->action)
    {
      case LEDSEQ_LOOP:
        state[led][prio] = 0;
        break;
      case LEDSEQ_STOP:
        state[led][prio] = LEDSEQ_STOP;
        updateActive(led);
        break;
      default:  //The step is a LED action and a time
        ledSet(led, step->value);
        if (step->action == 0)
          break;
        xTimerChangePeriod(xTimer, M2T(step->action), 0);
        xTimerStart(xTimer, 0);
        leave=true;
        break;
    }
    xSemaphoreGive(ledseqSem);
  }
}

//Utility functions
static int getPrio(const ledseq_t *seq)
{
  int prio;

  //Find the priority of the sequence
  for(prio=0; prio<SEQ_NUM; prio++)
    if(sequences[prio]==seq) return prio;

  return -1; //Invalid sequence
}

static void updateActive(led_t led)
{
  int prio;

  activeSeq[led]=LEDSEQ_STOP;
  ledSet(led, false);

  for(prio=0;prio<SEQ_NUM;prio++)
  {
    if (state[led][prio] != LEDSEQ_STOP)
    {
      activeSeq[led]=prio;
      break;
    }
  }
}
