/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#ifndef _PINKY_SIM_H_
#define _PINKY_SIM_H_

#include <IMemory.h>

/* Bits in PinkySimContext::xPSR */
#define APSR_N      (1 << 31)   /* Negative flag */
#define APSR_Z      (1 << 30)   /* Zero flag */
#define APSR_C      (1 << 29)   /* Carry flag */
#define APSR_V      (1 << 28)   /* Overflow flag */
#define IPSR_MASK   0x3F        /* Mask for exception number */
#define EPSR_T      (1 << 24)   /* Thumb mode flag */

/* Useful xPSR flag combinations for masking off at runtime. */
#define APSR_NZCV   (APSR_N | APSR_Z | APSR_C | APSR_V)
#define APSR_NZC    (APSR_N | APSR_Z | APSR_C)
#define APSR_NZ     (APSR_N | APSR_Z)
#define APSR_NC     (APSR_N | APSR_C)
#define APSR_ZC     (APSR_Z | APSR_C)

/* Register names / indices into PinkySimContext::R array for first 13 registers. */
#define R0  0
#define R1  1
#define R2  2
#define R3  3
#define R4  4
#define R5  5
#define R6  6
#define R7  7
#define R8  8
#define R9  9
#define R10 10
#define R11 11
#define R12 12
#define SP  13
#define LR  14
#define PC  15

typedef struct PinkySimContext
{
    IMemory* pMemory;
    uint32_t R[13];
    uint32_t lr;
    uint32_t spMain;
    uint32_t pc;
    uint32_t newPC;
    uint32_t xPSR;
} PinkySimContext;


/* Values that can be returned from the pinkSimStep() function. */
#define PINKYSIM_STEP_OK            0   /* Executed instruction successfully. */
#define PINKYSIM_STEP_UNDEFINED     1   /* Encountered undefined instruction. */
#define PINKYSIM_STEP_UNPREDICTABLE 2   /* Encountered instruction with unpredictable behaviour. */
#define PINKYSIM_STEP_HARDFAULT     3   /* Encountered instruction which generates hard fault. */


int pinkySimStep(PinkySimContext* pContext);


#endif /* _PINKY_SIM_H_ */
