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
#include <assert.h>
#include <common.h>
#include <pinkySimLogExe.h>
#include <MallocFailureInject.h>
#include <pinkySim.h>

#include <string.h>
#include <time.h>

// Additional details on a particular memory region
typedef struct MemInfoEntry
{
	uint32_t start; //!< Base address for the memory section
	uint32_t end; //!< Non-inclusive end address for memory section.
	const char* desc; //!< Description of the memory seciton.
} MemInfoEntry;

// Used to (potentially) give additional details on memory sections
static const struct MemInfoEntry* memInfo = NULL;
// Number of entires in memInfo
static uint32_t memInfoSize = 0;

// Memory sections specific to NXP LPC11U37 Chip
//  Not to be accessed directly, but rather an option that memInfo can be set to.
static const struct MemInfoEntry LPC11U37_MEM_INFO[] = 
{
	{0x00000000, 0x00020000, "128 kB on-chip flash"}, 	// --flash 0 131072
	{0x10000000, 0x10002000, "8 kB SRAM0"}, 		// --ram 268435456 8192
	{0x1FFF0000, 0x1FFF4000, "16 kB boot ROM"}, 		// --flash 536805376 16384
	{0x20000000, 0x20000800, "2 kB SRAM1"}, 		// --ram 536870912 2048
	{0x20004000, 0x20004800, "2 kB USB SRAM"},		// --ram 536887296 2048
	{0x40000000, 0x40004000, "I2C-bus"},			// --ram 1073741824 16384
	{0x40004000, 0x40008000, "WWDT"},			// --ram 1073758208 16384
	{0x40008000, 0x4000C000, "USART/SMART CARD"},		// --ram 1073774592 16384
	{0x4000C000, 0x40010000, "16-bit counter/timer 0"},	// --ram 1073790976 16384
	{0x40010000, 0x40014000, "16-bit counter/timer 1"},	// --ram 1073807360 16384
	{0x40014000, 0x40018000, "32-bit counter/timer 0"},	// --ram 1073823744 16384
	{0x40018000, 0x4001C000, "32-bit counter/timer 1"},	// --ram 1073840128 16384
	{0x4001C000, 0x40020000, "ADC"},			// --ram 1073856512 16384
	{0x40038000, 0x4003C000, "PMU"},			// --ram 1073971200 16384
	{0x4003C000, 0x40040000, "flash/EEPROM controller"},	// --ram 1073987584 16384
	{0x40040000, 0x40044000, "SSPO"},			// --ram 1074003968 16384
	{0x40044000, 0x40048000, "IOCON"},			// --ram 1074020352 16384
	{0x40048000, 0x4004C000, "system control"},		// --ram 1074036736 16384
	{0x4004C000, 0x40050000, "GPIO interrupts"},		// --ram 1074053120 16384
	{0x40058000, 0x4005C000, "SSP1"},			// --ram 1074102272 16384
	{0x4005C000, 0x40060000, "GPIO GROUP0 INT"},		// --ram 1074118656 16384
	{0x40060000, 0x40064000, "GPIO GROUP1 INT"},		// --ram 1074135040 16384
	{0x40080000, 0x40084000, "USB"},			// --ram 1074266112 16384
	{0x50000000, 0x50004000, "GPIO"},			// --ram 1342177280 16484
	{0xE0000000, 0xE0100000, "private peripheral bus"},	// --ram 3758096384 1048576
	{0xE0100000, 0xFFFFFFFF, "reserved"}
};

/**
 * \param addr Address to get description of.
 *
 * \return Description of given memory that given address falls under.
 */
const char* getMemInfo(uint32_t addr)
{
	const char* retval = "Unknown";
	size_t cnt = 0;

	for (cnt = 0; cnt < memInfoSize; cnt++)
	{
		if (addr >= memInfo[cnt].start && addr < memInfo[cnt].end)
		{
			retval = memInfo[cnt].desc; 
			break;
		}
	}

	return retval;
}

/* File storing csv representation of details regarding instructions exectued */
static FILE* exeLogCsvFile = NULL;
/* File storing an attempt to convert instructions executed during sim to 
    C-style code */
static FILE* exeLogCFile = NULL;
/* The number of tabs to insert before each line written to exeLogCsvFile */
static int exeLogCNumTabs = 0;

/**
 * Enable execution logging for current simulation.
 *
 * \param[in] chipType Chip Type info that is used to load memory info for
 * 	logging (i.e. additional details for accessed memory regions).
 */
void logExeEnable(const char* chipType)
{
	char tmp_str[128];
	time_t rawtime;
	size_t cnt = 0;

	if (!strcmp("LPC11U37", chipType))
	{
		memInfo = LPC11U37_MEM_INFO;
		memInfoSize = ARRAY_SIZE(LPC11U37_MEM_INFO);
	}

	time(&rawtime);

	// Open log for assembly details
	snprintf(tmp_str, ARRAY_SIZE(tmp_str), "exeLog_%020llu.csv", 
		(uint64_t)rawtime);
	exeLogCsvFile = fopen(tmp_str, "w");

	// Add first entry, which is header describing each column
	fprintf(exeLogCsvFile, " Entry Num, ");
	fprintf(exeLogCsvFile, "File Offset, ");
	fprintf(exeLogCsvFile, "  Raw Data, ");

	snprintf(tmp_str, ARRAY_SIZE(tmp_str), "Decoded Raw Data");
	for (cnt = strnlen(tmp_str, ARRAY_SIZE(tmp_str)); cnt < ARRAY_SIZE(tmp_str); cnt++)
	{
		fprintf(exeLogCsvFile, " ");
	}
	fprintf(exeLogCsvFile, "%s, ", tmp_str);

	fprintf(exeLogCsvFile, "  Step Num, ");

	fprintf(exeLogCsvFile, "Current PC, ");
	fprintf(exeLogCsvFile, "   Next PC, ");
	fprintf(exeLogCsvFile, "        SP, ");
	fprintf(exeLogCsvFile, "        LR, ");
	fprintf(exeLogCsvFile, "      XPSR, ");
	fprintf(exeLogCsvFile, "   PRIMASK, ");
	fprintf(exeLogCsvFile, "   CONTROL, ");

	for (cnt = 0; cnt < 13; cnt++)	
	{
		fprintf(exeLogCsvFile, "              Register % 2d, ", 
			(int)cnt);
	}

	fprintf(exeLogCsvFile, "\n");

	fflush(exeLogCsvFile);

	// Open file for logging of C style decomposition of simulation
	snprintf(tmp_str, ARRAY_SIZE(tmp_str), "exeLog_%020llu.c", 
		(uint64_t)rawtime);
	exeLogCFile = fopen(tmp_str, "w");

	fprintf(exeLogCFile, "/**\n");
	fprintf(exeLogCFile, " * This is an automatically generated file that "
		"attempts to take the data\n");
	fprintf(exeLogCFile, " *  obtained during a simulation run and "
		"attempts to create a C code\n");
	fprintf(exeLogCFile, " *  representation of the executed code.\n");
	fprintf(exeLogCFile, " */\n");
	fprintf(exeLogCFile, "void sim_run_%020llu()\n", (uint64_t)rawtime);
	fprintf(exeLogCFile, "{\n");
	logExeIncIndentCCode();

	fflush(exeLogCFile);
}

static const uint8_t MAX_DESC_STR_LEN = 128; //!< Maximum length of description
	//!< field in exe log.

/**
 * Add entry into log so we can review execution later.
 *
 * \param[in] context Current processor context (reg values, etc.)
 * \param offset Memory offset being affected.
 * \param rawData Raw Data at memory offset.
 * \param size Number of valid bytes in rawData.
 * \param[in] format Human readable description of entry.
 * \param arg A value identifying a variable arguments list initialized with va_start.
 *
 * \return None.
 */
static void logExeCsvEntry(const struct PinkySimContext* context, uint32_t offset, 
	uint32_t rawData, uint32_t size, const char* format, va_list arg)
{
	static uint32_t entryNum = 0;
	size_t cnt = 0;

	// Check if logging was enabled
	if (!exeLogCsvFile)
		return;

	fprintf(exeLogCsvFile, "% 10d, ", entryNum);
	fprintf(exeLogCsvFile, " 0x%08x, ", offset);
	switch(size)
	{
		case 1:
			fprintf(exeLogCsvFile, "      0x%02x, ", 0xFF&rawData);
			break;
		case 2:
			fprintf(exeLogCsvFile, "    0x%04x, ", 0xFFFF&rawData);
			break;
		case 4:
			fprintf(exeLogCsvFile, "0x%08x, ", 0xFFFFFFFF&rawData);
			break;
		default:
			fprintf(exeLogCsvFile, "Exception: invalidArgumentException for size %d", size);
			fflush(exeLogCsvFile);
			__throw(invalidArgumentException);
			break;
	}
	
	char desc[MAX_DESC_STR_LEN];
	desc[MAX_DESC_STR_LEN-1] = 0;
	vsnprintf(desc, MAX_DESC_STR_LEN, format, arg);
	for (cnt = strnlen(desc, MAX_DESC_STR_LEN); cnt < MAX_DESC_STR_LEN; cnt++)
	{
		fprintf(exeLogCsvFile, " ");
	}
	fprintf(exeLogCsvFile, "%s, ", desc);

	fprintf(exeLogCsvFile, "  % 8d, ", context->stepNum);

	fprintf(exeLogCsvFile, "0x%08x, ", context->pc);
	fprintf(exeLogCsvFile, "0x%08x, ", context->newPC);
	fprintf(exeLogCsvFile, "0x%08x, ", context->spMain);
	fprintf(exeLogCsvFile, "0x%08x, ", context->lr);
	fprintf(exeLogCsvFile, "0x%08x, ", context->xPSR);
	fprintf(exeLogCsvFile, "0x%08x, ", context->PRIMASK);
	fprintf(exeLogCsvFile, "0x%08x, ", context->CONTROL);

	for (cnt = 0; cnt < ARRAY_SIZE(context->R); cnt++)	
	{
		fprintf(exeLogCsvFile, "0x%08x (% 12d), ", context->R[cnt], context->R[cnt]);
	}

	fprintf(exeLogCsvFile, "\n");

	fflush(exeLogCsvFile);

	entryNum++;
}

/**
 * Add log entry for 16-bit instruction execution.
 *
 * \param[in] context Current processor context (reg values, etc.)
 * \param instr Raw instruction word.
 * \param[in] format Human readable description of instruction.
 * \param ... Additional argument like for printf().
 *
 * \return None.
 */
void logExeInstr16(const struct PinkySimContext* context, uint16_t instr, 
	const char* format, ...)
{
	va_list args;
	va_start(args, format);

	logExeCsvEntry(context, context->pc, instr, 2, format, args);

	va_end(args);
}

/**
 * Add log entry for 32-bit instruction execution.
 *
 * \param[in] context Current processor context (reg values, etc.)
 * \param instr1 Raw instruction low word.
 * \param instr2 Raw instruction high word.
 * \param[in] format Human readable description of instruction.
 * \param ... Additional argument like for printf().
 *
 * \return None.
 */
void logExeInstr32(const struct PinkySimContext* context, 
	uint16_t instr1, uint16_t instr2, const char* format, ...)
{
	uint32_t instr = (instr2<<16 | instr1);

	va_list args;
	va_start(args, format);

	logExeCsvEntry(context, context->pc, instr, 4, format, args);

	va_end(args);
}

/**
 * Add entry to exeLogCFile. It is assumed this is a C code style entry and
 *  should be indented to appropriate depth, based on exeLogCNumTabs.
 *
 * \param[in] format Human readable description of instruction.
 * \param ... Additional argument like for printf().
 *
 * \return None.
 */
void logExeCCode(const char* format, ...) {
	static int needs_indent = 1;

	if (needs_indent) {
		for (int cnt = 0; cnt < exeLogCNumTabs; cnt++) {
			fprintf(exeLogCFile, "\t");
		}

		needs_indent = 0;
	}

	va_list args;
	char str[256];
	va_start(args, format);

	str[sizeof(str)-1] = 0;
	vsnprintf(str, sizeof(str), format, args);
	fprintf(exeLogCFile, "%s", str);

	va_end(args);

	if (format[strnlen(format, 2048)-1] == '\n') {
		needs_indent = 1;
	}
	
	fflush(exeLogCFile);
}

/**
 * Increase exeLogCNumTabs.
 *
 * \return None.
 */
void logExeIncIndentCCode() {
	if (exeLogCNumTabs < 256) {
		exeLogCNumTabs++;
	} else {
		fprintf(exeLogCFile, "/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
		fprintf(exeLogCFile, " * Attempt made to raise exeLogCNumTabs above 255.\n");
		fprintf(exeLogCFile, " *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/\n");
	}
}

/**
 * Decrease exeLogCNumTabs.
 *
 * \return None.
 */
void logExeDecIndentCCode() {
	if (exeLogCNumTabs) {
		exeLogCNumTabs--;
	} else {
		fprintf(exeLogCFile, "/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
		fprintf(exeLogCFile, " * Attempt made to lower exeLogCNumTabs below zero.\n");
		fprintf(exeLogCFile, " *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/\n");
	}
}
