/*  Copyright (C) 2017  Gregory Gluszek (https://github.com/greggersaurus)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#ifndef _PINKY_SIM_LOGEXE_H_
#define _PINKY_SIM_LOGEXE_H_

// Forwar declaration
struct PinkySimContext;

void logExeEnable(const char* chipType);

const char* logExeGetMemInfo(uint32_t addr);

void logExeInstr16(const struct PinkySimContext* context, uint16_t instr, 
	const char* format, ...);
void logExeInstr32(const struct PinkySimContext* context, 
	uint16_t instr1, uint16_t instr2, const char* format, ...);

void logExeIncIndentCStyle();
void logExeDecIndentCStyle();

//TODO: to keep track of instructions that last affected conds, to be used by logExeCStyleVerbose();
//void logExeSetCondInstr(uint32_t cond, const char* format, ...);
//const char* logExeGetCondInstr();

void logExeCStyleVerbose(const char* format, ...);

//TODO: return true if value if register value was computed via constant(s) only. Can be used to keep register description strings reduced for further computations against other constants
int logExeRegHasConstVal(uint32_t regNum);
//TODO: return true if address falls into constant valued memory space (i.e. flash where compiled code is stored) and reading will return constant value
int logExeIsConstMem(uint32_t addr);

void logExeSetRegValStr(uint32_t regNum, uint32_t cond, const char* format, ...); 
const char* logExeGetRegValStr(uint32_t regNum);
void logExeSetRegCmtStr(uint32_t regNum, uint32_t cond, const char* format, ...); 
const char* logExeGetRegCmtStr(uint32_t regNum); 
void logExePushRegStrs(uint32_t regNum);
void logExePopRegStrs(uint32_t regNum);

void logExeSetCondValStr(uint32_t cond, const char* format, ...);
const char* logExeGetCondValStr(uint32_t cond);
void logExeSetCondCmtStr(uint32_t cond, const char* format, ...);
const char* logExeGetCondCmtStr(uint32_t cond);

void logExeCStyleSimplified(const char* format, ...);

//TODO: to close down file and deallocate, etc.
//void logExeCleanup();

#endif /* _PINKY_SIM_LOGEXE_H_ */
