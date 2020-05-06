/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2013 Bitcraze AB
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
 * usec_time.c - microsecond-resolution timer and timestamps.
 */

#include "usec_time.h"

#include "nvicconf.h"
#include "stm32fxxx.h"

static uint32_t usecTimerHighCount;

void initUsecTimer(void)
{
  usecTimerHighCount = 0;

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  //Enable the Timer
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  // APB1 clock is /4, but APB clock inputs to timers are doubled when
  // the APB clock runs slower than /1, so APB1 timers see a /2 clock
  TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / (1000 * 1000) / 2;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_TRACE_TIM_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  DBGMCU_APB1PeriphConfig(DBGMCU_TIM7_STOP, ENABLE);
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM7, ENABLE);
}

uint64_t usecTimestamp(void)
{
  uint32_t high0;
  __atomic_load(&usecTimerHighCount, &high0, __ATOMIC_SEQ_CST);
  uint32_t low = TIM7->CNT;
  uint32_t high;
  __atomic_load(&usecTimerHighCount, &high, __ATOMIC_SEQ_CST);

  // There was no increment in between
  if (high == high0)
  {
    return (((uint64_t)high) << 16) + low;
  }
  // There was an increment, but we don't expect another one soon
  return (((uint64_t)high) << 16) + TIM7->CNT;
}

void __attribute__((used)) TIM7_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM7, TIM_IT_Update);

  __sync_fetch_and_add(&usecTimerHighCount, 1);
}
