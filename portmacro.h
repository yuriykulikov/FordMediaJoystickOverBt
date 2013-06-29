/* This file has been prepared for Doxygen automatic documentation generation.*/
/*
 * Copyright (C) 2012 Yuriy Kulikov
 *      Universitaet Erlangen-Nuernberg
 *      LS Informationstechnik (Kommunikationselektronik)
 *      Support email: Yuriy.Kulikov.87@googlemail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PORTMACRO_H
#define PORTMACRO_H
#include "avr_compiler.h"

#define QUEUE_MAX_LEN 10

volatile uint32_t tick;
/*-----------------------------------------------------------
 * Port specific definitions.  
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

#define TIMER1_PREDIVISOR_1024 0x05
#define TIMER1_PREDIVISOR_256 0x04
#define TIMER1_PREDIVISOR_64 0x03
#define TIMER1_PREDIVISOR_8 0x02
#define TIMER1_PREDIVISOR_1 0x01
#define TICK_FREQUENCY 1000

/* Type definitions. */
#define portCHAR        char
#define portFLOAT       float
#define portDOUBLE      double
#define portLONG        long
#define portSHORT       int
#define portSTACK_TYPE    unsigned portCHAR
#define portBASE_TYPE    char

#define portDISABLE_INTERRUPTS()    asm volatile ( "cli" :: );
#define portENABLE_INTERRUPTS()     asm volatile ( "sei" :: );
/*-----------------------------------------------------------*/

/* Architecture specifics. */
#define portNOP()                    asm volatile ( "nop" );
/*-----------------------------------------------------------*/

void configureTickTimer (void);

#endif /* PORTMACRO_H */

