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
const char* getMemInfo(uint32_t addr);
void logExeInstr16(const struct PinkySimContext* context, uint16_t instr, 
	const char* format, ...);
void logExeInstr32(const struct PinkySimContext* context, 
	uint16_t instr1, uint16_t instr2, const char* format, ...);
void logExeCCode(const char* format, ...);
void logExeIncIndentCCode();
void logExeDecIndentCCode();
//TODO: to close down file and deallocate, etc.
//void cleanup();

#endif /* _PINKY_SIM_LOGEXE_H_ */
