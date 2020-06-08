/**
*
 * ESP-Drone Firmware
 * 
 * Copyright 2019-2020  Espressif Systems (Shanghai) 
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
 *
 * cfassert.h - Assert macro
 */

#include "console.h"

#ifndef __CFASSERT_H__
#define __CFASSERT_H__

#define ASSERT(e)  if (e) ; \
        else assertFail( #e, __FILE__, __LINE__ )

#ifdef DEBUG_EP2
#define IF_DEBUG_ASSERT(e)  if (e) ; \
        else assertFail( #e, __FILE__, __LINE__ )
#else
#define IF_DEBUG_ASSERT(e)
#endif

#define ASSERT_FAILED() assertFail( "", __FILE__, __LINE__ )

/**
 * Assert handler function
 */
void assertFail(char *exp, char *file, int line);
/**
 * Print assert snapshot data
 */
void printAssertSnapshotData();
/**
 * Store assert snapshot data to be read at startup if a reset is triggered (watchdog)
 */
void storeAssertFileData(const char *file, int line);
/**
 * Store hardfault data to be read at startup if a reset is triggered (watchdog)
 * Line information can be printed using:
 * > make gdb
 * gdb> info line *0x<PC>
 */
void storeAssertHardfaultData(
    unsigned int r0,
    unsigned int r1,
    unsigned int r2,
    unsigned int r3,
    unsigned int r12,
    unsigned int lr,
    unsigned int pc,
    unsigned int psr);

/**
 * Store assert data to be read at startup if a reset is triggered (watchdog)
 */
void storeAssertTextData(const char *text);

#endif //__CFASSERT_H__
