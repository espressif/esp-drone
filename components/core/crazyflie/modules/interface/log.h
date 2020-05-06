/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012-2019 BitCraze AB
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
 * log.h - Dynamic log system
 */

#ifndef __LOG_H__
#define __LOG_H__

#include <stdbool.h>
#include <stdint.h>

/* Public functions */
void logInit(void);
bool logTest(void);

/* Internal access of log variables */
int logGetVarId(char* group, char* name);
int logGetType(int varid);
void logGetGroupAndName(int varid, char** group, char** name);
void* logGetAddress(int varid);
uint8_t logVarSize(int type);
float logGetFloat(int varid);
int logGetInt(int varid);
unsigned int logGetUint(int varid);

/* Basic log structure */
struct log_s {
  uint8_t type;
  char * name;
  void * address;
};

/* Possible variable types */
#define LOG_UINT8  1
#define LOG_UINT16 2
#define LOG_UINT32 3
#define LOG_INT8   4
#define LOG_INT16  5
#define LOG_INT32  6
#define LOG_FLOAT  7
#define LOG_FP16   8

typedef uint8_t (*logAcquireUInt8)(uint32_t timestamp, void* data);
typedef uint16_t (*logAcquireUInt16)(uint32_t timestamp, void* data);
typedef uint32_t (*logAcquireUInt32)(uint32_t timestamp, void* data);
typedef int8_t (*logAcquireInt8)(uint32_t timestamp, void* data);
typedef int16_t (*logAcquireInt16)(uint32_t timestamp, void* data);
typedef int32_t (*logAcquireInt32)(uint32_t timestamp, void* data);
typedef float (*logAcquireFloat)(uint32_t timestamp, void* data);

typedef struct {
  union {
    logAcquireUInt8 acquireUInt8;
    logAcquireUInt16 acquireUInt16;
    logAcquireUInt32 acquireUInt32;
    logAcquireInt8 acquireInt8;
    logAcquireInt16 acquireInt16;
    logAcquireInt32 acquireInt32;
    logAcquireFloat aquireFloat;
  };

  void* data;
} logByFunction_t;

/* Internal defines */
#define LOG_GROUP 0x80
#define LOG_BY_FUNCTION 0x40
#define LOG_START 1
#define LOG_STOP  0

/* Macros */

#ifndef UNIT_TEST_MODE

#define LOG_ADD(TYPE, NAME, ADDRESS) \
   { .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

#define LOG_ADD_BY_FUNCTION(TYPE, NAME, ADDRESS) \
   { .type = TYPE | LOG_BY_FUNCTION, .name = #NAME, .address = (void*)(ADDRESS), },

#define LOG_ADD_GROUP(TYPE, NAME, ADDRESS) \
   { \
  .type = TYPE, .name = #NAME, .address = (void*)(ADDRESS), },

#define LOG_GROUP_START(NAME)  \
  static const struct log_s __logs_##NAME[] __attribute__((section(".log." #NAME), used)) = { \
  LOG_ADD_GROUP(LOG_GROUP | LOG_START, NAME, 0x0)

//#define LOG_GROUP_START_SYNC(NAME, LOCK) LOG_ADD_GROUP(LOG_GROUP | LOG_START, NAME, LOCK);

#define LOG_GROUP_STOP(NAME) \
  LOG_ADD_GROUP(LOG_GROUP | LOG_STOP, stop_##NAME, 0x0) \
  };

#else // UNIT_TEST_MODE

// Empty defines when running unit tests
#define LOG_ADD(TYPE, NAME, ADDRESS)
#define LOG_ADD_BY_FUNCTION(TYPE, NAME, ADDRESS)
#define LOG_ADD_GROUP(TYPE, NAME, ADDRESS)
#define LOG_GROUP_START(NAME)
#define LOG_GROUP_STOP(NAME)

#endif // UNIT_TEST_MODE

#endif /* __LOG_H__ */
