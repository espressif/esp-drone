/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 * @file static_mem.h
 * @brief Utility macros to create static memory for OS objects such as
 * queues, semaphores and so on.
 *
 * @copyright Copyright (c) 2019
 *
 */

#pragma once

/**
 * @brief Creation of queues using static memory.
 *
 * STATIC_MEM_QUEUE_ALLOC() and STATIC_MEM_QUEUE_CREATE() are used togehter to set up
 * and create a queue. STATIC_MEM_QUEUE_ALLOC() defines a number of static variables,
 * that are required by the queue. STATIC_MEM_QUEUE_CREATE() creates the OS queue object
 * using the previously defined variables.
 *
 * Example:
 * static xQueueHandle myQueue;
 * STATIC_MEM_QUEUE_ALLOC(myQueue, 5, sizeof(int));
 * // ...
 * void init() {
 *   myQueue = STATIC_MEM_QUEUE_CREATE(myQueue);
 *   // myQueue can now be used in OS calls
 * }
 *
 * Note: the queue handle may have the same name as used in the macros, but it is
 * not necessary.
 */

/**
 * @brief Creates a number of variables required for a queue using static memory.
 *
 * All variables created by this macreo are named using the NAME parameter as a base.
 *
 * Example:
 * STATIC_MEM_QUEUE_ALLOC(myQueue, 5, sizeof(int));
 *
 * expands to:
 * // static const int osSys_myQueueLength = 5;
 * // static const int osSys_myQueueItemSize = sizeof(int);
 * // static uint8_t osSys_myQueueStorage[5 * sizeof(int)];
 * // static StaticQueue_t osSys_myQueueSMgm;
 *
 * @param NAME - the name of the queue handle (xQueueHandle). The name is also used as
 * base name for the other variables that are required.
 * @param LENGTH - the length of the queue (in items)
 * @param ITEM_SIZE - the size of the items in the queue
 */
#define STATIC_MEM_QUEUE_ALLOC(NAME, LENGTH, ITEM_SIZE)\
  static const int osSys_ ## NAME ## Length = (LENGTH); \
  static const int osSys_ ## NAME ## ItemSize = (ITEM_SIZE); \
  static uint8_t osSys_ ## NAME ## Storage[(LENGTH) * (ITEM_SIZE)]; \
  static StaticQueue_t osSys_ ## NAME ## Mgm;

/**
 * @brief Creates a queue using static memory
 *
 * The queue is created under the assumption that variables have been created
 * using the STATIC_MEM_QUEUE_ALLOC() macro
 *
 * Example:
 * STATIC_MEM_QUEUE_CREATE(myQueue);
 * // Exoands to
 * // xQueueCreateStatic(osSys_myQueueLength, osSys_myQueueItemSize, osSys_myQueueStorage, &osSys_myQueueMgm);
 *
 * @param NAME - the name of the queue handle
 */
#define STATIC_MEM_QUEUE_CREATE(NAME) xQueueCreateStatic(osSys_ ## NAME ## Length, osSys_ ## NAME ## ItemSize, osSys_ ## NAME ## Storage, &osSys_ ## NAME ## Mgm)


/**
 * @brief Creation of tasks using static memory.
 *
 * STATIC_MEM_TASK_ALLOC() and STATIC_MEM_TASK_CREATE() are used together to
 * allocate buffers and create a task. STATIC_MEM_TASK_ALLOC() defines the
 * required memory and variables while STATIC_MEM_TASK_CREATE() creats
 * the task. The NAME is used as a base name for the necessary variables and
 * does not have to be the same as FUNCTION or TASK_NAME, but that works as well.
 *
 * Example:
 * static TaskHandle_t taskHandle;
 * STATIC_MEM_TASK_ALLOC(myTask, 100);
 *
 * void taskFcn(void *arg) {
 *   // Do stuff...
 * }
 *
 * void init() {
 *  taskHandle = STATIC_MEM_TASK_CREATE(myTask, taskFcn, "MY_TASK", NULL, 1);
 *  // ...
 * }
 */

/**
 * @brief Allocate variables and buffers for a task using static memory.
 *
 * @param NAME A name used as base name for the variables that are created
 * @param STACK_DEPTH The stack depth in nr of StackType_t entries.
 */
#define STATIC_MEM_TASK_ALLOC(NAME, STACK_DEPTH) \
  static const int osSys_ ## NAME ## StackDepth = (STACK_DEPTH); \
  static StackType_t osSys_ ## NAME ## StackBuffer[(STACK_DEPTH)]; \
  static StaticTask_t osSys_ ## NAME ## TaskBuffer;

/**
 * @brief Create a task using static memory
 *
 * The task is created under the assumption that STATIC_MEM_TASK_ALLOC() has been
 * used to define the required variables and buffers.
 *
 * @param NAME A name used as base name for the variables, same name that was used in STATIC_MEM_TASK_ALLOC()
 * @param FUNCTION The function that implements the task
 * @param TASK_NAME A descriptive name for the task
 * @param PARAMETERS Passed on as argument to the function implementing the task
 * @param PRIORITY The task priority
 */
#define STATIC_MEM_TASK_CREATE(NAME, FUNCTION, TASK_NAME, PARAMETERS, PRIORITY) xTaskCreateStatic((FUNCTION), (TASK_NAME), osSys_ ## NAME ## StackDepth, (PARAMETERS), (PRIORITY), osSys_ ## NAME ## StackBuffer, &osSys_ ## NAME ## TaskBuffer)
