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
	uint32_t end; //!< Inclusive end address for memory section.
	const char* desc; //!< Description of the memory seciton.
	int constVal; //!< Set to true if memory has constant value.
} MemInfoEntry;

// Used to (potentially) give additional details on memory sections
static const struct MemInfoEntry* memInfo = NULL;
// Number of entires in memInfo
static uint32_t memInfoSize = 0;

// Memory sections specific to NXP LPC11U37 Chip
//  Not to be accessed directly, but rather an option that memInfo can be set to.
static const struct MemInfoEntry LPC11U37_MEM_INFO[] = 
{
	{0x00000000, 0x00020000-1, "128 kB on-chip flash", TRUE}, // --flash 0 131072
	{0x10000000, 0x10002000-1, "8 kB SRAM0", FALSE}, // --ram 268435456 8192
	{0x1FFF0000, 0x1FFF4000-1, "16 kB boot ROM", TRUE}, // --flash 536805376 16384
	{0x20000000, 0x20000800-1, "2 kB SRAM1", FALSE}, // --ram 536870912 2048
	{0x20004000, 0x20004800-1, "2 kB USB SRAM", FALSE}, // --ram 536887296 2048

	//{0x40000000, 0x40004000-1, "I2C-bus"}, // --ram 1073741824 16384
	{0x40000000, 0x40000004-1, "I2C: CONSET", FALSE},
	{0x40000004, 0x40000008-1, "I2C: STAT", FALSE},
	{0x40000008, 0x4000000C-1, "I2C: DAT", FALSE},
	{0x4000000C, 0x40000010-1, "I2C: ADR0", FALSE},
	{0x40000010, 0x40000014-1, "I2C: SCLH", FALSE},
	{0x40000014, 0x40000018-1, "I2C: SCLL", FALSE},
	{0x40000018, 0x4000001C-1, "I2C: CONCLR", FALSE},
	{0x4000001C, 0x40000020-1, "I2C: MCCTRL", FALSE},
	{0x40000020, 0x40000024-1, "I2C: ADR1", FALSE},
	{0x40000024, 0x40000028-1, "I2C: ADR2", FALSE},
	{0x40000028, 0x4000002C-1, "I2C: ADR3", FALSE},
	{0x4000002C, 0x40000030-1, "I2C: DATA_BUFFER", FALSE},
	{0x40000030, 0x40000034-1, "I2C: MASK0", FALSE},
	{0x40000034, 0x40000038-1, "I2C: MASK1", FALSE},
	{0x40000038, 0x4000003C-1, "I2C: MASK2", FALSE},
	{0x4000003C, 0x40000040-1, "I2C: MASK3", FALSE},

	//{0x40004000, 0x40008000-1, "WWDT"}, // --ram 1073758208 16384
	{0x40004000, 0x40004004-1, "WWDT: MOD", FALSE},
	{0x40004004, 0x40004008-1, "WWDT: TC", FALSE},
	{0x40004008, 0x4000400C-1, "WWDT: FEED", FALSE},
	{0x4000400C, 0x40004010-1, "WWDT: TV", FALSE},
	{0x40004010, 0x40004014-1, "WWDT: CLKSEL", FALSE},
	{0x40004014, 0x40004018-1, "WWDT: WARNINT", FALSE},
	{0x40004018, 0x4000401C-1, "WWDT: WINDOW", FALSE},

	//{0x40008000, 0x4000C000-1, "USART/SMART CARD"}, // --ram 1073774592 16384
	{0x40008000, 0x40008004-1, "USART: RBR/THR or DLL", FALSE},
	{0x40008004, 0x40008008-1, "USART: DLM or IER", FALSE},
	{0x40008008, 0x4000800C-1, "USART: IIR or FCR", FALSE},
	{0x4000800C, 0x40008010-1, "USART: LCR", FALSE},
	{0x40008010, 0x40008014-1, "USART: MCR", FALSE},
	{0x40008014, 0x40008018-1, "USART: LSR", FALSE},
	{0x40008018, 0x4000801C-1, "USART: MSR", FALSE},
	{0x4000801C, 0x40008020-1, "USART: SCR", FALSE},
	{0x40008020, 0x40008024-1, "USART: ACR", FALSE},
	{0x40008024, 0x40008028-1, "USART: ICR", FALSE},
	{0x40008028, 0x4000802C-1, "USART: FDR", FALSE},
	{0x4000802C, 0x40008030-1, "USART: OSR", FALSE},
	{0x40008030, 0x40008034-1, "USART: TER", FALSE},
	{0x40008040, 0x40008044-1, "USART: HDEN", FALSE},
	{0x40008048, 0x4000804C-1, "USART: SCICTRL", FALSE},
	{0x4000804C, 0x40008050-1, "USART: RS485CTRL", FALSE},
	{0x40008050, 0x40008054-1, "USART: RS485ADRMATCH", FALSE},
	{0x40008054, 0x40008058-1, "USART: RS485DLY", FALSE},
	{0x40008058, 0x4000805C-1, "USART: SYNCCTRL", FALSE},

	//{0x4000C000, 0x40010000-1, "16-bit counter/timer 0"}, // --ram 1073790976 16384
	{0x4000C000, 0x4000C004-1, "CT16B0: IR", FALSE},
	{0x4000C004, 0x4000C008-1, "CT16B0: TCR", FALSE},
	{0x4000C008, 0x4000C00C-1, "CT16B0: TC", FALSE},
	{0x4000C00C, 0x4000C010-1, "CT16B0: PR", FALSE},
	{0x4000C010, 0x4000C014-1, "CT16B0: PC", FALSE},
	{0x4000C014, 0x4000C018-1, "CT16B0: MCR", FALSE},
	{0x4000C018, 0x4000C01C-1, "CT16B0: MR0", FALSE},
	{0x4000C01C, 0x4000C020-1, "CT16B0: MR1", FALSE},
	{0x4000C020, 0x4000C024-1, "CT16B0: MR2", FALSE},
	{0x4000C024, 0x4000C028-1, "CT16B0: MR3", FALSE},
	{0x4000C028, 0x4000C02C-1, "CT16B0: CCR", FALSE},
	{0x4000C02C, 0x4000C030-1, "CT16B0: CR0", FALSE},
	{0x4000C034, 0x4000C038-1, "CT16B0: CR1", FALSE},
	{0x4000C03C, 0x4000C040-1, "CT16B0: EMR", FALSE},
	{0x4000C070, 0x4000C074-1, "CT16B0: CTCR", FALSE},
	{0x4000C074, 0x4000C078-1, "CT16B0: PWMC", FALSE},

	//{0x40010000, 0x40014000-1, "16-bit counter/timer 1"}, // --ram 1073807360 16384
	{0x40010000, 0x40010004-1, "CT16B1: IR", FALSE},
	{0x40010004, 0x40010008-1, "CT16B1: TCR", FALSE},
	{0x40010008, 0x4001000C-1, "CT16B1: TC", FALSE},
	{0x4001000C, 0x40010010-1, "CT16B1: PR", FALSE},
	{0x40010010, 0x40010014-1, "CT16B1: PC", FALSE},
	{0x40010014, 0x40010018-1, "CT16B1: MCR", FALSE},
	{0x40010018, 0x4001001C-1, "CT16B1: MR0", FALSE},
	{0x4001001C, 0x40010020-1, "CT16B1: MR1", FALSE},
	{0x40010020, 0x40010024-1, "CT16B1: MR2", FALSE},
	{0x40010024, 0x40010028-1, "CT16B1: MR3", FALSE},
	{0x40010028, 0x4001002C-1, "CT16B1: CCR", FALSE},
	{0x4001002C, 0x40010030-1, "CT16B1: CR0", FALSE},
	{0x40010034, 0x40010038-1, "CT16B1: CR1", FALSE},
	{0x4001003C, 0x40010040-1, "CT16B1: EMR", FALSE},
	{0x40010070, 0x40010074-1, "CT16B1: CTCR", FALSE},
	{0x40010074, 0x40010078-1, "CT16B1: PWMC", FALSE},

	//{0x40014000, 0x40018000-1, "32-bit counter/timer 0"}, // --ram 1073823744 16384
	{0x40014000, 0x40014004-1, "CT32B0: IR", FALSE},
	{0x40014004, 0x40014008-1, "CT32B0: TCR", FALSE},
	{0x40014008, 0x4001400C-1, "CT32B0: TC", FALSE},
	{0x4001400C, 0x40014010-1, "CT32B0: PR", FALSE},
	{0x40014010, 0x40014014-1, "CT32B0: PC", FALSE},
	{0x40014014, 0x40014018-1, "CT32B0: MCR", FALSE},
	{0x40014018, 0x4001401C-1, "CT32B0: MR0", FALSE},
	{0x4001401C, 0x40014020-1, "CT32B0: MR1", FALSE},
	{0x40014020, 0x40014024-1, "CT32B0: MR2", FALSE},
	{0x40014024, 0x40014028-1, "CT32B0: MR3", FALSE},
	{0x40014028, 0x4001402C-1, "CT32B0: CCR", FALSE},
	{0x4001402C, 0x40014030-1, "CT32B0: CR0", FALSE},
	{0x40014034, 0x40014038-1, "CT32B0: CR1", FALSE},
	{0x4001403C, 0x40014040-1, "CT32B0: EMR", FALSE},
	{0x40014070, 0x40014074-1, "CT32B0: CTCR", FALSE},
	{0x40014074, 0x40014078-1, "CT32B0: PWMC", FALSE},

	//{0x40018000, 0x4001C000-1, "32-bit counter/timer 1"}, // --ram 1073840128 16384
	{0x40018000, 0x40018004-1, "CT32B1: IR", FALSE},
	{0x40018004, 0x40018008-1, "CT32B1: TCR", FALSE},
	{0x40018008, 0x4001800C-1, "CT32B1: TC", FALSE},
	{0x4001800C, 0x40018010-1, "CT32B1: PR", FALSE},
	{0x40018010, 0x40018014-1, "CT32B1: PC", FALSE},
	{0x40018014, 0x40018018-1, "CT32B1: MCR", FALSE},
	{0x40018018, 0x4001801C-1, "CT32B1: MR0", FALSE},
	{0x4001801C, 0x40018020-1, "CT32B1: MR1", FALSE},
	{0x40018020, 0x40018024-1, "CT32B1: MR2", FALSE},
	{0x40018024, 0x40018028-1, "CT32B1: MR3", FALSE},
	{0x40018028, 0x4001802C-1, "CT32B1: CCR", FALSE},
	{0x4001802C, 0x40018030-1, "CT32B1: CR0", FALSE},
	{0x40018034, 0x40018038-1, "CT32B1: CR1", FALSE},
	{0x4001803C, 0x40018040-1, "CT32B1: EMR", FALSE},
	{0x40018070, 0x40018074-1, "CT32B1: CTCR", FALSE},
	{0x40018074, 0x40018078-1, "CT32B1: PWMC", FALSE},

	//{0x4001C000, 0x40020000-1, "ADC"}, // --ram 1073856512 16384
	{0x4001C000, 0x4001C004-1, "ADC: CR", FALSE},
	{0x4001C004, 0x4001C008-1, "ADC: GDR", FALSE},
	{0x4001C00C, 0x4001C010-1, "ADC: INTEN", FALSE},
	{0x4001C010, 0x4001C014-1, "ADC: DR0", FALSE},
	{0x4001C014, 0x4001C018-1, "ADC: DR1", FALSE},
	{0x4001C018, 0x4001C01C-1, "ADC: DR2", FALSE},
	{0x4001C01C, 0x4001C020-1, "ADC: DR3", FALSE},
	{0x4001C020, 0x4001C024-1, "ADC: DR4", FALSE},
	{0x4001C024, 0x4001C028-1, "ADC: DR5", FALSE},
	{0x4001C028, 0x4001C02C-1, "ADC: DR6", FALSE},
	{0x4001C02C, 0x4001C030-1, "ADC: DR7", FALSE},
	{0x4001C030, 0x4001C034-1, "ADC: STAT", FALSE},

	//{0x40038000, 0x4003C000-1, "PMU"}, // --ram 1073971200 16384
	{0x40038000, 0x40038004-1, "PMU: PCON", FALSE},
	{0x40038004, 0x40038008-1, "PMU: GPREG0", FALSE},
	{0x40038008, 0x4003800C-1, "PMU: GPREG1", FALSE},
	{0x4003800C, 0x40038010-1, "PMU: GPREG4", FALSE},
	{0x40038010, 0x40038014-1, "PMU: GPREG3", FALSE},
	{0x40038014, 0x40038018-1, "PMU: GPREG4", FALSE},

	//{0x4003C000, 0x40040000-1, "flash/EEPROM controller"}, // --ram 1073987584 16384
	{0x4003C010, 0x4003C014-1, "flash/EEPROM controller: FLASHCFG", FALSE},	

	//{0x40040000, 0x40044000-1, "SSPO"}, // --ram 1074003968 16384
	{0x40040000, 0x40040004-1, "SSP/SPI0: Ctrl Reg 0", FALSE},
	{0x40040004, 0x40040008-1, "SSP/SPI0: Ctrl Reg 1", FALSE},
	{0x40040008, 0x4004000C-1, "SSP/SPI0: Data Reg", FALSE},
	{0x4004000C, 0x40040010-1, "SSP/SPI0: Status Reg", FALSE},
	{0x40040010, 0x40040014-1, "SSP/SPI0: Clk Prescale Reg", FALSE},
	{0x40040014, 0x40040018-1, "SSP/SPI0: IMSC", FALSE},
	{0x40040018, 0x4004001C-1, "SSP/SPI0: RIS", FALSE},
	{0x4004001C, 0x40040020-1, "SSP/SPI0: MIS", FALSE},
	{0x40040020, 0x40040024-1, "SSP/SPI0: ICR", FALSE},

	//{0x40044000, 0x40048000-1, "IOCON"}, // --ram 1074020352 16384
	{0x40044000, 0x40044004-1, "IOCON: RESET_PIO0_0", FALSE},
	{0x40044004, 0x40044008-1, "IOCON: PIO0_1", FALSE},
	{0x40044008, 0x4004400C-1, "IOCON: PIO0_2", FALSE},
	{0x4004400C, 0x40044010-1, "IOCON: PIO0_3", FALSE},
	{0x40044010, 0x40044014-1, "IOCON: PIO0_4", FALSE},
	{0x40044014, 0x40044018-1, "IOCON: PIO0_5", FALSE},
	{0x40044018, 0x4004401C-1, "IOCON: PIO0_6", FALSE},
	{0x4004401C, 0x40044020-1, "IOCON: PIO0_7", FALSE},
	{0x40044020, 0x40044024-1, "IOCON: PIO0_8", FALSE},
	{0x40044024, 0x40044028-1, "IOCON: PIO0_9", FALSE},
	{0x40044028, 0x4004402C-1, "IOCON: SWCLK_PIO0_10", FALSE},
	{0x4004402C, 0x40044030-1, "IOCON: TDI_PIO0_11", FALSE},
	{0x40044030, 0x40044034-1, "IOCON: TMS_PIO0_12", FALSE},
	{0x40044034, 0x40044038-1, "IOCON: TDO_PIO0_13", FALSE},
	{0x40044038, 0x4004403C-1, "IOCON: TRST_PIO0_14", FALSE},
	{0x4004403C, 0x40044040-1, "IOCON: SWDIO_PIO0_15", FALSE},
	{0x40044040, 0x40044044-1, "IOCON: PIO0_16", FALSE},
	{0x40044044, 0x40044048-1, "IOCON: PIO0_17", FALSE},
	{0x40044048, 0x4004404C-1, "IOCON: PIO0_18", FALSE},
	{0x4004404C, 0x40044050-1, "IOCON: PIO0_19", FALSE},
	{0x40044050, 0x40044054-1, "IOCON: PIO0_20", FALSE},
	{0x40044054, 0x40044058-1, "IOCON: PIO0_21", FALSE},
	{0x40044058, 0x4004405C-1, "IOCON: PIO0_22", FALSE},
	{0x4004405C, 0x40044060-1, "IOCON: PIO0_23", FALSE},
	{0x40044060, 0x40044064-1, "IOCON: PIO1_0", FALSE},
	{0x40044064, 0x40044068-1, "IOCON: PIO1_1", FALSE},
	{0x40044068, 0x4004406C-1, "IOCON: PIO1_2", FALSE},
	{0x4004406C, 0x40044070-1, "IOCON: PIO1_3", FALSE},
	{0x40044070, 0x40044074-1, "IOCON: PIO1_4", FALSE},
	{0x40044074, 0x40044078-1, "IOCON: PIO1_5", FALSE},
	{0x40044078, 0x4004407C-1, "IOCON: PIO1_6", FALSE},
	{0x4004407C, 0x40044080-1, "IOCON: PIO1_7", FALSE},
	{0x40044080, 0x40044084-1, "IOCON: PIO1_8", FALSE},
	{0x40044084, 0x40044088-1, "IOCON: PIO1_9", FALSE},
	{0x40044088, 0x4004408C-1, "IOCON: PIO1_10", FALSE},
	{0x4004408C, 0x40044090-1, "IOCON: PIO1_11", FALSE},
	{0x40044090, 0x40044094-1, "IOCON: PIO1_12", FALSE},
	{0x40044094, 0x40044098-1, "IOCON: PIO1_13", FALSE},
	{0x40044098, 0x4004409C-1, "IOCON: PIO1_14", FALSE},
	{0x4004409C, 0x400440A0-1, "IOCON: PIO1_15", FALSE},
	{0x400440A0, 0x400440A4-1, "IOCON: PIO1_16", FALSE},
	{0x400440A4, 0x400440A8-1, "IOCON: PIO1_17", FALSE},
	{0x400440A8, 0x400440AC-1, "IOCON: PIO1_18", FALSE},
	{0x400440AC, 0x400440B0-1, "IOCON: PIO1_19", FALSE},
	{0x400440B0, 0x400440B4-1, "IOCON: PIO1_20", FALSE},
	{0x400440B4, 0x400440B8-1, "IOCON: PIO1_21", FALSE},
	{0x400440B8, 0x400440BC-1, "IOCON: PIO1_22", FALSE},
	{0x400440BC, 0x400440C0-1, "IOCON: PIO1_23", FALSE},
	{0x400440C0, 0x400440C4-1, "IOCON: PIO1_24", FALSE},
	{0x400440C4, 0x400440C8-1, "IOCON: PIO1_25", FALSE},
	{0x400440C8, 0x400440CC-1, "IOCON: PIO1_26", FALSE},
	{0x400440CC, 0x400440D0-1, "IOCON: PIO1_27", FALSE},
	{0x400440D0, 0x400440D4-1, "IOCON: PIO1_28", FALSE},
	{0x400440D4, 0x400440D8-1, "IOCON: PIO1_29", FALSE},
	{0x400440DC, 0x400440E0-1, "IOCON: PIO1_31", FALSE},

	//{0x40048000, 0x4004C000-1, "system control"}, // --ram 1074036736 16384
	{0x40048000, 0x40048004-1, "system control: SYSMEMREMAP", FALSE},
	{0x40048004, 0x40048008-1, "system control: PRESETCTRL", FALSE},
	{0x40048008, 0x4004800C-1, "system control: SYSPLLCTRL", FALSE},
	{0x4004800C, 0x40048010-1, "system control: SYSPLLSTAT", FALSE},
	{0x40048010, 0x40048014-1, "system control: USBPLLCTRL", FALSE},
	{0x40048014, 0x40048018-1, "system control: USBPLLSTAT", FALSE},
	{0x40048020, 0x40048024-1, "system control: SYSOSCCTRL", FALSE},
	{0x40048024, 0x40048028-1, "system control: WDTOSCCTRL", FALSE},
	{0x40048028, 0x4004802C-1, "system control: IRCCTRL", FALSE},
	{0x40048030, 0x40048034-1, "system control: SYSRSTSTAT", FALSE},
	{0x40048040, 0x40048044-1, "system control: SYSPLLCLKSEL", FALSE},
	{0x40048044, 0x40048048-1, "system control: SYSPLLCLKUEN", FALSE},
	{0x40048070, 0x40048074-1, "system control: MAINCLKSEL", FALSE},
	{0x40048074, 0x40048078-1, "system control: MAINCLKUEN", FALSE},
	{0x40048078, 0x4004807C-1, "system control: SYSAHBCLKDIV", FALSE},
	{0x40048080, 0x40048084-1, "system control: SYSAHBCLKCTRL", FALSE},
	{0x40048094, 0x40048098-1, "system control: SSP0CLKDIV", FALSE},
	{0x40048098, 0x4004809C-1, "system control: UARTCLKDIV", FALSE},
	{0x4004809C, 0x400480A0-1, "system control: SSP1CLKDIV", FALSE},
	{0x400480C0, 0x400480C4-1, "system control: USBCLKSEL", FALSE},
	{0x400480C4, 0x400480C8-1, "system control: USBCLKUEN", FALSE},
	{0x400480C8, 0x400480CC-1, "system control: USBCLKDIV", FALSE},
	{0x400480E0, 0x400480E4-1, "system control: CLKOUTSEL", FALSE},
	{0x400480E4, 0x400480E8-1, "system control: CLKOUTUEN", FALSE},
	{0x400480E8, 0x400480EC-1, "system control: CLKOUTDIV", FALSE},
	{0x40048100, 0x40048104-1, "system control: PIOPORCAP0", FALSE},
	{0x40048104, 0x40048108-1, "system control: PIOPORCAP1", FALSE},
	{0x40048150, 0x40048154-1, "system control: BODCTRL", FALSE},
	{0x40048154, 0x40048158-1, "system control: SYSTCKCAL", FALSE},
	{0x40048170, 0x40048174-1, "system control: IRQLATENCY", FALSE},
	{0x40048174, 0x40048178-1, "system control: NMISRC", FALSE},
	{0x40048178, 0x4004817C-1, "system control: PINTSEL0", FALSE},
	{0x4004817C, 0x40048180-1, "system control: PINTSEL1", FALSE},
	{0x40048180, 0x40048184-1, "system control: PINTSEL2", FALSE},
	{0x40048184, 0x40048188-1, "system control: PINTSEL3", FALSE},
	{0x40048188, 0x4004818C-1, "system control: PINTSEL4", FALSE},
	{0x4004818C, 0x40048190-1, "system control: PINTSEL5", FALSE},
	{0x40048190, 0x40048194-1, "system control: PINTSEL6", FALSE},
	{0x40048194, 0x40048198-1, "system control: PINTSEL7", FALSE},
	{0x40048198, 0x4004819C-1, "system control: USBCLKCTRL", FALSE},
	{0x4004819C, 0x400481A0-1, "system control: USBCLKST", FALSE},
	{0x40048204, 0x40048208-1, "system control: STARTERP0", FALSE},
	{0x40048214, 0x40048218-1, "system control: STARTERP1", FALSE},
	{0x40048230, 0x40048234-1, "system control: PDSLEEPCFG", FALSE},
	{0x40048234, 0x40048238-1, "system control: PDWAKECFG", FALSE},
	{0x40048238, 0x4004823C-1, "system control: PDRUNCFG", FALSE},
	{0x400483F4, 0x400483F8-1, "system control: DEVICE_ID", FALSE},

	//{0x4004C000, 0x40050000-1, "GPIO interrupts"}, // --ram 1074053120 16384
	{0x4004C000, 0x4004C004-1, "GPIO interrupts: ISEL", FALSE},
	{0x4004C004, 0x4004C008-1, "GPIO interrupts: IENR", FALSE},
	{0x4004C008, 0x4004C00C-1, "GPIO interrupts: SIENR", FALSE},
	{0x4004C00C, 0x4004C010-1, "GPIO interrupts: CIENR", FALSE},
	{0x4004C010, 0x4004C014-1, "GPIO interrupts: IENF", FALSE},
	{0x4004C014, 0x4004C018-1, "GPIO interrupts: SIENF", FALSE},
	{0x4004C018, 0x4004C01C-1, "GPIO interrupts: CIENF", FALSE},
	{0x4004C01C, 0x4004C020-1, "GPIO interrupts: RISE", FALSE},
	{0x4004C020, 0x4004C024-1, "GPIO interrupts: FALL", FALSE},
	{0x4004C024, 0x4004C028-1, "GPIO interrupts: IST", FALSE},

	//{0x40058000, 0x4005C000-1, "SSP1"}, // --ram 1074102272 16384
	{0x40058000, 0x40058004-1, "SSP/SPI1: Ctrl Reg 0", FALSE},
	{0x40058004, 0x40058008-1, "SSP/SPI1: Ctrl Reg 1", FALSE},
	{0x40058008, 0x4005800C-1, "SSP/SPI1: Data Reg", FALSE},
	{0x4005800C, 0x40058010-1, "SSP/SPI1: Status Reg", FALSE},
	{0x40058010, 0x40058014-1, "SSP/SPI1: Clk Prescale Reg", FALSE},
	{0x40058014, 0x40058018-1, "SSP/SPI1: IMSC", FALSE},
	{0x40058018, 0x4005801C-1, "SSP/SPI1: RIS", FALSE},
	{0x4005801C, 0x40058020-1, "SSP/SPI1: MIS", FALSE},
	{0x40058020, 0x40058024-1, "SSP/SPI1: ICR", FALSE},

	//{0x4005C000, 0x40060000-1, "GPIO GROUP0 INT"}, // --ram 1074118656 16384
	{0x4005C000, 0x4005C004-1, "GPIO GROUP0 INT: CTRL", FALSE},
	{0x4005C020, 0x4005C024-1, "GPIO GROUP0 INT: PORT_POL0", FALSE},
	{0x4005C024, 0x4005C028-1, "GPIO GROUP0 INT: PORT_POL1", FALSE},
	{0x4005C028, 0x4005C02C-1, "GPIO GROUP0 INT: PORT_POL2", FALSE},
	{0x4005C040, 0x4005C044-1, "GPIO GROUP0 INT: PORT_ENA0", FALSE},
	{0x4005C044, 0x4005C04C-1, "GPIO GROUP0 INT: PORT_ENA1", FALSE},

	//{0x40060000, 0x40064000-1, "GPIO GROUP1 INT"}, // --ram 1074135040 16384
	{0x40060000, 0x40060004-1, "GPIO GROUP1 INT: CTRL", FALSE},
	{0x40060020, 0x40060024-1, "GPIO GROUP1 INT: PORT_POL0", FALSE},
	{0x40060024, 0x40060028-1, "GPIO GROUP1 INT: PORT_POL1", FALSE},
	{0x40060028, 0x4006002C-1, "GPIO GROUP1 INT: PORT_POL2", FALSE},
	{0x40060040, 0x40060044-1, "GPIO GROUP1 INT: PORT_ENA0", FALSE},
	{0x40060044, 0x4006004C-1, "GPIO GROUP1 INT: PORT_ENA1", FALSE},

	//{0x40080000, 0x40084000-1, "USB"}, // --ram 1074266112 16384
	{0x40080000, 0x40080004-1, "USB: DEVCMDSTAT", FALSE},
	{0x40080004, 0x40080008-1, "USB: INFO", FALSE},
	{0x40080008, 0x4008000C-1, "USB: EPLISTSTART", FALSE},
	{0x4008000C, 0x40080010-1, "USB: DATABUFFSTART", FALSE},
	{0x40080010, 0x40080014-1, "USB: LPM", FALSE},
	{0x40080014, 0x40080018-1, "USB: EPSKIP", FALSE},
	{0x40080018, 0x4008001C-1, "USB: EPINUSE", FALSE},
	{0x4008001C, 0x40080020-1, "USB: EPBUFCFG", FALSE},
	{0x40080020, 0x40080024-1, "USB: INTSTAT", FALSE},
	{0x40080024, 0x40080028-1, "USB: INTEN", FALSE},
	{0x40080028, 0x4008002C-1, "USB: INTSETSTAT", FALSE},
	{0x4008002C, 0x40080030-1, "USB: INTROUTING", FALSE},
	{0x40080030, 0x40080034-1, "USB: EPTOGGLE", FALSE},

	//{0x50000000, 0x50004000-1, "GPIO"}, // --ram 1342177280 16484
	{0x50000000, 0x50000000, "GPIO: P0_0 PBYTE", FALSE},
	{0x50000001, 0x50000001, "GPIO: P0_1 PBYTE", FALSE},
	{0x50000002, 0x50000002, "GPIO: P0_2 PBYTE", FALSE},
	{0x50000003, 0x50000003, "GPIO: P0_3 PBYTE", FALSE},
	{0x50000004, 0x50000004, "GPIO: P0_4 PBYTE", FALSE},
	{0x50000005, 0x50000005, "GPIO: P0_5 PBYTE", FALSE},
	{0x50000006, 0x50000006, "GPIO: P0_6 PBYTE", FALSE},
	{0x50000007, 0x50000007, "GPIO: P0_7 PBYTE", FALSE},
	{0x50000008, 0x50000008, "GPIO: P0_8 PBYTE", FALSE},
	{0x50000009, 0x50000009, "GPIO: P0_9 PBYTE", FALSE},
	{0x5000000A, 0x5000000A, "GPIO: P0_10 PBYTE", FALSE},
	{0x5000000B, 0x5000000B, "GPIO: P0_11 PBYTE", FALSE},
	{0x5000000C, 0x5000000C, "GPIO: P0_12 PBYTE", FALSE},
	{0x5000000D, 0x5000000D, "GPIO: P0_13 PBYTE", FALSE},
	{0x5000000E, 0x5000000E, "GPIO: P0_14 PBYTE", FALSE},
	{0x5000000F, 0x5000000F, "GPIO: P0_15 PBYTE", FALSE},
	{0x50000010, 0x50000010, "GPIO: P0_16 PBYTE", FALSE},
	{0x50000011, 0x50000011, "GPIO: P0_17 PBYTE", FALSE},
	{0x50000012, 0x50000012, "GPIO: P0_18 PBYTE", FALSE},
	{0x50000013, 0x50000013, "GPIO: P0_19 PBYTE", FALSE},
	{0x50000014, 0x50000014, "GPIO: P0_20 PBYTE", FALSE},
	{0x50000015, 0x50000015, "GPIO: P0_21 PBYTE", FALSE},
	{0x50000016, 0x50000016, "GPIO: P0_22 PBYTE", FALSE},
	{0x50000017, 0x50000017, "GPIO: P0_23 PBYTE", FALSE},
	{0x50000020, 0x50000020, "GPIO: P1_0 PBYTE", FALSE},
	{0x50000021, 0x50000021, "GPIO: P1_1 PBYTE", FALSE},
	{0x50000022, 0x50000022, "GPIO: P1_2 PBYTE", FALSE},
	{0x50000023, 0x50000023, "GPIO: P1_3 PBYTE", FALSE},
	{0x50000024, 0x50000024, "GPIO: P1_4 PBYTE", FALSE},
	{0x50000025, 0x50000025, "GPIO: P1_5 PBYTE", FALSE},
	{0x50000026, 0x50000026, "GPIO: P1_6 PBYTE", FALSE},
	{0x50000027, 0x50000027, "GPIO: P1_7 PBYTE", FALSE},
	{0x50000028, 0x50000028, "GPIO: P1_8 PBYTE", FALSE},
	{0x50000029, 0x50000029, "GPIO: P1_9 PBYTE", FALSE},
	{0x5000002A, 0x5000002A, "GPIO: P1_10 PBYTE", FALSE},
	{0x5000002B, 0x5000002B, "GPIO: P1_11 PBYTE", FALSE},
	{0x5000002C, 0x5000002C, "GPIO: P1_12 PBYTE", FALSE},
	{0x5000002D, 0x5000002D, "GPIO: P1_13 PBYTE", FALSE},
	{0x5000002E, 0x5000002E, "GPIO: P1_14 PBYTE", FALSE},
	{0x5000002F, 0x5000002F, "GPIO: P1_15 PBYTE", FALSE},
	{0x50000030, 0x50000030, "GPIO: P1_16 PBYTE", FALSE},
	{0x50000031, 0x50000031, "GPIO: P1_17 PBYTE", FALSE},
	{0x50000032, 0x50000032, "GPIO: P1_18 PBYTE", FALSE},
	{0x50000033, 0x50000033, "GPIO: P1_19 PBYTE", FALSE},
	{0x50000034, 0x50000034, "GPIO: P1_20 PBYTE", FALSE},
	{0x50000035, 0x50000035, "GPIO: P1_21 PBYTE", FALSE},
	{0x50000036, 0x50000036, "GPIO: P1_22 PBYTE", FALSE},
	{0x50000037, 0x50000037, "GPIO: P1_23 PBYTE", FALSE},
	{0x50000038, 0x50000038, "GPIO: P1_24 PBYTE", FALSE},
	{0x50000039, 0x50000039, "GPIO: P1_25 PBYTE", FALSE},
	{0x5000003A, 0x5000003A, "GPIO: P1_26 PBYTE", FALSE},
	{0x5000003B, 0x5000003B, "GPIO: P1_27 PBYTE", FALSE},
	{0x5000003C, 0x5000003C, "GPIO: P1_28 PBYTE", FALSE},
	{0x5000003D, 0x5000003D, "GPIO: P1_29 PBYTE", FALSE},
	{0x5000003E, 0x5000003E, "GPIO: P1_30 PBYTE", FALSE},
	{0x5000003F, 0x5000003F, "GPIO: P1_31 PBYTE", FALSE},
	{0x50001000, 0x50001004-1, "GPIO: P0_0 PWORD", FALSE}, 
	{0x50001004, 0x50001008-1, "GPIO: P0_1 PWORD", FALSE}, 
	{0x50001008, 0x5000100C-1, "GPIO: P0_2 PWORD", FALSE}, 
	{0x5000100C, 0x50001010-1, "GPIO: P0_3 PWORD", FALSE}, 
	{0x50001010, 0x50001014-1, "GPIO: P0_4 PWORD", FALSE}, 
	{0x50001014, 0x50001018-1, "GPIO: P0_5 PWORD", FALSE}, 
	{0x50001018, 0x5000101C-1, "GPIO: P0_6 PWORD", FALSE}, 
	{0x5000101C, 0x50001020-1, "GPIO: P0_7 PWORD", FALSE}, 
	{0x50001020, 0x50001024-1, "GPIO: P0_8 PWORD", FALSE}, 
	{0x50001024, 0x50001028-1, "GPIO: P0_9 PWORD", FALSE}, 
	{0x50001028, 0x5000102C-1, "GPIO: P0_10 PWORD", FALSE}, 
	{0x5000102C, 0x50001030-1, "GPIO: P0_11 PWORD", FALSE}, 
	{0x50001030, 0x50001034-1, "GPIO: P0_12 PWORD", FALSE}, 
	{0x50001034, 0x50001038-1, "GPIO: P0_13 PWORD", FALSE}, 
	{0x50001038, 0x5000103C-1, "GPIO: P0_14 PWORD", FALSE}, 
	{0x5000103C, 0x50001040-1, "GPIO: P0_15 PWORD", FALSE}, 
	{0x50001040, 0x50001044-1, "GPIO: P0_16 PWORD", FALSE}, 
	{0x50001044, 0x50001048-1, "GPIO: P0_17 PWORD", FALSE}, 
	{0x50001048, 0x5000104C-1, "GPIO: P0_18 PWORD", FALSE}, 
	{0x5000104C, 0x50001050-1, "GPIO: P0_19 PWORD", FALSE}, 
	{0x50001050, 0x50001054-1, "GPIO: P0_20 PWORD", FALSE}, 
	{0x50001054, 0x50001058-1, "GPIO: P0_21 PWORD", FALSE}, 
	{0x50001058, 0x5000105C-1, "GPIO: P0_22 PWORD", FALSE}, 
	{0x5000105C, 0x50001060-1, "GPIO: P0_23 PWORD", FALSE}, 
	{0x50001080, 0x50001084-1, "GPIO: P1_0 PWORD", FALSE}, 
	{0x50001084, 0x50001088-1, "GPIO: P1_1 PWORD", FALSE}, 
	{0x50001088, 0x5000108C-1, "GPIO: P1_2 PWORD", FALSE}, 
	{0x5000108C, 0x50001090-1, "GPIO: P1_3 PWORD", FALSE}, 
	{0x50001090, 0x50001094-1, "GPIO: P1_4 PWORD", FALSE}, 
	{0x50001094, 0x50001098-1, "GPIO: P1_5 PWORD", FALSE}, 
	{0x50001098, 0x5000109C-1, "GPIO: P1_6 PWORD", FALSE}, 
	{0x5000109C, 0x500010A0-1, "GPIO: P1_7 PWORD", FALSE}, 
	{0x500010A0, 0x500010A4-1, "GPIO: P1_8 PWORD", FALSE}, 
	{0x500010A4, 0x500010A8-1, "GPIO: P1_9 PWORD", FALSE}, 
	{0x500010A8, 0x500010AC-1, "GPIO: P1_10 PWORD", FALSE}, 
	{0x500010AC, 0x500010B0-1, "GPIO: P1_11 PWORD", FALSE}, 
	{0x500010B0, 0x500010B4-1, "GPIO: P1_12 PWORD", FALSE}, 
	{0x500010B4, 0x500010B8-1, "GPIO: P1_13 PWORD", FALSE}, 
	{0x500010B8, 0x500010BC-1, "GPIO: P1_14 PWORD", FALSE}, 
	{0x500010BC, 0x500010C0-1, "GPIO: P1_15 PWORD", FALSE}, 
	{0x500010C0, 0x500010C4-1, "GPIO: P1_16 PWORD", FALSE}, 
	{0x500010C4, 0x500010C8-1, "GPIO: P1_17 PWORD", FALSE}, 
	{0x500010C8, 0x500010CC-1, "GPIO: P1_18 PWORD", FALSE}, 
	{0x500010CC, 0x500010D0-1, "GPIO: P1_19 PWORD", FALSE}, 
	{0x500010D0, 0x500010D4-1, "GPIO: P1_20 PWORD", FALSE}, 
	{0x500010D4, 0x500010D8-1, "GPIO: P1_21 PWORD", FALSE}, 
	{0x500010D8, 0x500010DC-1, "GPIO: P1_22 PWORD", FALSE}, 
	{0x500010DC, 0x500010E0-1, "GPIO: P1_23 PWORD", FALSE}, 
	{0x500010E0, 0x500010E4-1, "GPIO: P1_24 PWORD", FALSE}, 
	{0x500010E4, 0x500010E8-1, "GPIO: P1_25 PWORD", FALSE}, 
	{0x500010E8, 0x500010EC-1, "GPIO: P1_26 PWORD", FALSE}, 
	{0x500010EC, 0x500010F0-1, "GPIO: P1_27 PWORD", FALSE}, 
	{0x500010F0, 0x500010F4-1, "GPIO: P1_28 PWORD", FALSE}, 
	{0x500010F4, 0x500010F8-1, "GPIO: P1_29 PWORD", FALSE}, 
	{0x500010F8, 0x500010FC-1, "GPIO: P1_30 PWORD", FALSE}, 
	{0x500010FC, 0x50001100-1, "GPIO: P1_31 PWORD", FALSE}, 
	{0x50002000, 0x50002004-1, "GPIO: DIR0", FALSE}, 
	{0x50002004, 0x50002008-1, "GPIO: DIR1", FALSE}, 
	{0x50002080, 0x50002084-1, "GPIO: MASK0", FALSE}, 
	{0x50002084, 0x50002088-1, "GPIO: MASK1", FALSE}, 
	{0x50002100, 0x50002104-1, "GPIO: PIN0", FALSE}, 
	{0x50002104, 0x50002108-1, "GPIO: PIN1", FALSE}, 
	{0x50002180, 0x50002184-1, "GPIO: MPIN0", FALSE}, 
	{0x50002184, 0x50002188-1, "GPIO: MPIN1", FALSE}, 
	{0x50002200, 0x50002204-1, "GPIO: SET0", FALSE}, 
	{0x50002204, 0x50002208-1, "GPIO: SET1", FALSE}, 
	{0x50002280, 0x50002284-1, "GPIO: CLR0", FALSE}, 
	{0x50002284, 0x50002288-1, "GPIO: CLR1", FALSE}, 
	{0x50002300, 0x50002304-1, "GPIO: NOT0", FALSE}, 
	{0x50002304, 0x50002308-1, "GPIO: NOT1", FALSE}, 

	//{0xE0000000, 0xE0100000-1, "private peripheral bus"}, // --ram 3758096384 1048576
	{0xE000E010, 0xE000E014-1, "SysTick timer: SYST_CSR", FALSE},
	{0xE000E014, 0xE000E018-1, "SysTick timer: SYST_RVR", FALSE},
	{0xE000E018, 0xE000E01C-1, "SysTick timer: SYST_CVR", FALSE},
	{0xE000E01C, 0xE000E020-1, "SysTick timer: SYST_CALIB", FALSE},

	{0xE000E100, 0xE000E104-1, "NVIC: ISER0", FALSE},
	{0xE000E180, 0xE000E184-1, "NVIC: ICER0", FALSE},
	{0xE000E200, 0xE000E204-1, "NVIC: ISPR0", FALSE},
	{0xE000E280, 0xE000E284-1, "NVIC: ICPR0", FALSE},
	{0xE000E300, 0xE000E304-1, "NVIC: IABR0", FALSE},
	{0xE000E400, 0xE000E404-1, "NVIC: IPR0", FALSE},
	{0xE000E404, 0xE000E408-1, "NVIC: IPR1", FALSE},
	{0xE000E408, 0xE000E40C-1, "NVIC: IPR2", FALSE},
	{0xE000E40C, 0xE000E410-1, "NVIC: IPR3", FALSE},
	{0xE000E410, 0xE000E414-1, "NVIC: IPR4", FALSE},
	{0xE000E414, 0xE000E418-1, "NVIC: IPR5", FALSE},
	{0xE000E418, 0xE000E41C-1, "NVIC: IPR6", FALSE},
	{0xE000E41C, 0xE000E420-1, "NVIC: IPR7", FALSE},

	{0xE000ED00, 0xE000ED04-1, "SCB: CPUID", FALSE},
	{0xE000ED04, 0xE000ED08-1, "SCB: ICSR", FALSE},
	{0xE000ED0C, 0xE000ED10-1, "SCB: AIRCR", FALSE},
	{0xE000ED10, 0xE000ED14-1, "SCB: SCR", FALSE},
	{0xE000ED14, 0xE000ED18-1, "SCB: CCR", FALSE},
	{0xE000ED1C, 0xE000ED20-1, "SCB: SHPR2", FALSE},
	{0xE000ED20, 0xE000ED24-1, "SCB: SHPR3", FALSE},

	{0xE0100000, 0xFFFFFFFF-1, "reserved", FALSE}
};

/**
 * \param addr Address to get details for.
 *
 * \return Entry details related addr. Or NULL if none exist.
 */
static const MemInfoEntry* getMemInfoEntry(uint32_t addr) 
{
	for (size_t cnt = 0; cnt < memInfoSize; cnt++)
	{
		if (addr >= memInfo[cnt].start && addr <= memInfo[cnt].end)
		{
			return &memInfo[cnt]; 
		}
	}

	return NULL;
}

/**
 * \param addr Address to get description of.
 *
 * \return Description of given memory that given address falls under.
 */
const char* logExeGetMemInfo(uint32_t addr)
{
	const MemInfoEntry* entry = getMemInfoEntry(addr);

//TODO: it would seem this is broken for some cases...?

	if (entry) 
	{
		return entry->desc;
	}

	return "Unknown";
}

/**
 * \return True if address falls into constant valued memory space (i.e. flash 
 *	where compiled code is stored) and reading will return constant value.
 */
int logExeIsConstMem(uint32_t addr)
{
	const MemInfoEntry* entry = getMemInfoEntry(addr);

	if (entry) 
	{
		return entry->constVal;
	}

	return FALSE;
}

/* File storing csv representation of details regarding instructions exectued */
static FILE* exeLogCsvFile = NULL;
/* File storing an attempt to convert instructions executed during sim to 
    C-style code */
static FILE* exeLogCFile = NULL;
/* Like exeLogCFile, except intermediate read/write access to registers are 
	combined to final memory access or conditional instructions */
static FILE* exeLogCSimpliedFile = NULL;
/* The number of tabs to insert before each line written to exeLogCsvFile */
static int exeLogCNumTabs = 0;

static const uint32_t MAX_DESC_STR_LEN = 128 + 32; //!< Maximum length of description
	//!< field in exe log.

//TODO: this too short...?
static const uint32_t MAX_REG_STR_LEN = 1024;
static const uint32_t MAX_REG_STR_STACK_SIZE = 32;
static const uint32_t NUM_REGS = 16; //! We do not track how PC gets updated. SP = 13, LR = 14

uint32_t regStackDepths[NUM_REGS]; //!< Keeps tracks of how many pushes more pushes
	//!< than pops have occurred for specified register.

// Descriptive strings for various register and conditional settings
typedef struct DescStrs 
{
	char reg[NUM_REGS][MAX_REG_STR_STACK_SIZE][MAX_REG_STR_LEN]; //!< String representing what is stored in each Register.
	char apsr_n[MAX_REG_STR_LEN]; //!< Action that last updated Negative Flag in ASPR register
        char apsr_z[MAX_REG_STR_LEN]; //!< Action that last updated Zero Flag in ASPR register
        char apsr_c[MAX_REG_STR_LEN]; //!< Action that last updated Carry Flag in ASPR register
        char apsr_v[MAX_REG_STR_LEN]; //!< Action that last updated Overflow Flag in ASPR register
} DescStrs;

static DescStrs valStrs; //!< Describes how each register or conditional got its 
	//! value (i.e. starting from a memory read).
static DescStrs cmtStrs; //!< Comments related to how each register or conditional 
	//! got its value.

typedef struct HasConstVals 
{
	int reg[NUM_REGS][MAX_REG_STR_STACK_SIZE]; //!< If is set to TRUE
		//!< it indcates the value contained in the register is a constant.
		//!< FALSE indicates value in register is based on values that may
		//!< change in different circumstances (i.e. different settings 
		//!< external to device).
	int apsr_n; //!< Same as reg, except for action that last updated Negative Flag in ASPR register
        int apsr_z; //!< Same as reg, except for action that last updated Zero Flag in ASPR register
        int apsr_c; //!< Same as reg, except for action that last updated Carry Flag in ASPR register
        int apsr_v; //!< Same as reg, except for action that last updated Overflow Flag in ASPR register
} HasConstVals;

static HasConstVals hasConstVals;

/**
 * Enable execution logging for current simulation.
 *
 * \param[in] chipType Chip Type info that is used to load memory info for
 * 	logging (i.e. additional details for accessed memory regions).
 */
void logExeEnable(const char* chipType)
{
	char tmp_str[MAX_DESC_STR_LEN];
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

	fflush(exeLogCFile);

	// Open file for logging of C style decomposition of simulation
	snprintf(tmp_str, ARRAY_SIZE(tmp_str), "exeLog_%020llu.simplified.c", 
		(uint64_t)rawtime);
	exeLogCSimpliedFile = fopen(tmp_str, "w");

	fprintf(exeLogCSimpliedFile, "/**\n");
	fprintf(exeLogCSimpliedFile, " * This is an automatically generated file that "
		"attempts to take the data\n");
	fprintf(exeLogCSimpliedFile, " *  obtained during a simulation run and "
		"attempts to create a C code\n");
	fprintf(exeLogCSimpliedFile, " *  representation of the executed code.\n");
	fprintf(exeLogCSimpliedFile, " *  Intermediate register reads and writes\n");
	fprintf(exeLogCSimpliedFile, " *  are combined to final memory reads and\n");
	fprintf(exeLogCSimpliedFile, " *  writes, or conditional statements, making\n");
	fprintf(exeLogCSimpliedFile, " *  this a simplified C conversion.\n");
	fprintf(exeLogCSimpliedFile, " */\n");
	fprintf(exeLogCSimpliedFile, "void sim_run_%020llu()\n", (uint64_t)rawtime);
	fprintf(exeLogCSimpliedFile, "{\n");

	fflush(exeLogCSimpliedFile);

	// Initialize valStrs to default values
	memset(&valStrs, 0, sizeof(valStrs));
	memset(&cmtStrs, 0, sizeof(cmtStrs));

	uint32_t stack_depth = 0;
	for (uint32_t reg_num = 0; reg_num < 13; reg_num++) 
	{
		for (stack_depth = 0; stack_depth < MAX_REG_STR_STACK_SIZE-1; stack_depth++) 
		{
			logExeSetRegCmtStr(reg_num, 0, ""); 
			logExeSetRegValStr(reg_num, 0, FALSE, "Init"); 
			logExePushRegStrs(reg_num);
		}
		logExeSetRegValStr(reg_num, 0, FALSE, "Init"); 
		for (stack_depth = 0; stack_depth < MAX_REG_STR_STACK_SIZE-1; stack_depth++) 
		{
			logExePopRegStrs(reg_num);
		}
	}

	for (stack_depth = 0; stack_depth < MAX_REG_STR_STACK_SIZE-1; stack_depth++) 
	{
		logExeSetRegValStr(13, 0, FALSE, "SP"); 
		logExePushRegStrs(13);
		logExeSetRegValStr(14, 0, FALSE, "LR"); 
		logExePushRegStrs(14);
		logExeSetRegValStr(15, 0, FALSE, "PC"); 
		logExePushRegStrs(15);
	}
	logExeSetRegValStr(13, 0, FALSE, "SP"); 
	logExeSetRegValStr(14, 0, FALSE, "LR"); 
	logExeSetRegValStr(15, 0, FALSE, "PC"); 
	for (stack_depth = 0; stack_depth < MAX_REG_STR_STACK_SIZE-1; stack_depth++) 
	{
		logExePopRegStrs(13);
		logExePopRegStrs(14);
		logExePopRegStrs(15);
	}

	logExeSetCondValStr(APSR_NZCV, FALSE, "Init");

	logExeIncIndentCStyle();

	// All registers start with non-const values (they should be set before
	//  they are referenced).
	memset(&hasConstVals, 0, sizeof(hasConstVals));
	// All conditionals start with const values (they may may be referenced
	//  before they are initially set).
	hasConstVals.apsr_n = TRUE;
        hasConstVals.apsr_z = TRUE;
        hasConstVals.apsr_c = TRUE;
        hasConstVals.apsr_v = TRUE;
}

static uint32_t csvEntryNum = 0;

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
	size_t cnt = 0;

	// Check if logging was enabled
	if (!exeLogCsvFile)
		return;

	fprintf(exeLogCsvFile, "% 10d, ", csvEntryNum);
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
	
	//TODO: use vfprintf...
	char desc[MAX_DESC_STR_LEN];
	desc[MAX_DESC_STR_LEN-1] = 0;
	vsnprintf(desc, MAX_DESC_STR_LEN, format, arg);
	for (cnt = strnlen(desc, MAX_DESC_STR_LEN); cnt < MAX_DESC_STR_LEN; cnt++)
	{
		fprintf(exeLogCsvFile, " ");
	}
	fprintf(exeLogCsvFile, "%s, ", desc);

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

	csvEntryNum++;
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
void logExeCStyleVerbose(const char* format, ...) {
	static int needs_indent = 1;

	// Check if logging was enabled
	if (!exeLogCsvFile)
		return;

	if (needs_indent) {
		for (int cnt = 0; cnt < exeLogCNumTabs; cnt++) {
			fprintf(exeLogCFile, "\t");
		}

		needs_indent = 0;
	}

	va_list args;
	char str[256];
	va_start(args, format);

	//TODO: use vfprintf...
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
 * Increase depth at which next C style line starts (i.e. as if entering curly
 *  brace section).
 *
 * \return None.
 */
void logExeIncIndentCStyle() {
	if (exeLogCNumTabs < 256) {
		exeLogCNumTabs++;
	} else {
		logExeCStyleVerbose("ERROR(%d): %s: Attempt made to raise exeLogCNumTabs above 255\n", 
			csvEntryNum, __func__);
		logExeCStyleSimplified("ERROR(%d): %s: Attempt made to raise exeLogCNumTabs above 255\n", 
			csvEntryNum, __func__);
	}
}

/**
 * Decrease depth at which next C style line starts (i.e. as if leaving curly
 *  brace section).
 *
 * \return None.
 */
void logExeDecIndentCStyle() {
	if (exeLogCNumTabs) {
		exeLogCNumTabs--;
	} else {
		logExeCStyleVerbose("ERROR(%d): %s: Attempt made to raise exeLogCNumTabs above below zero\n", 
			csvEntryNum, __func__);
		logExeCStyleSimplified("ERROR(%d): %s: Attempt made to raise exeLogCNumTabs above below zero\n", 
			csvEntryNum, __func__);
	}
}

/**
 * Generic function for updating the conditional related descriptive strings 
 *  in DescStrs.
 *
 * \param descStr[in] Pointer to DescStrs to be updated (i.e. may pertain to
 *	value description strings or comment description strings).
 * \param cond Defines which conditional bits are affected and which
 *  conditional strings should be updated. Use APSR_* macros.
 * \param str String describing action that updated condition.
 *
 * \return None.
 */
static void logExeSetCondDescStrOnly(const struct DescStrs* descStrs, 
	uint32_t cond, const char* str)
{
	if (cond & APSR_N)
	{
		memcpy((void*)descStrs->apsr_n, str, MAX_REG_STR_LEN);
	}
	if (cond & APSR_Z)
	{
		memcpy((void*)descStrs->apsr_z, str, MAX_REG_STR_LEN);
	}
	if (cond & APSR_C)
	{
		memcpy((void*)descStrs->apsr_c, str, MAX_REG_STR_LEN);
	}
	if (cond & APSR_V)
	{
		memcpy((void*)descStrs->apsr_v, str, MAX_REG_STR_LEN);
	}
}

/**
 * Update the description string for the given register. Associated conditional strings
 *  are also updated if need be.
 *
 * \param descStr[in] Pointer to DescStrs to be updated (i.e. may pertain to
 *	value description strings or comment description strings).
 * \param regNum 0 based register number that defines which string is to be
 *  updated. 
 * \param cond Defines which conditional bits are affected and which
 *  conditional strings should be updated. Use APSR_* macros.
 * \param[in] format Human readable description of entry.
 * \param arg A value identifying a variable arguments list initialized with va_start.
 * 
 * \return None.
 */
static void logExeSetRegDescStr(const struct DescStrs* descStrs, 
	uint32_t regNum, uint32_t cond, const char* format, va_list arg)
{
	char reg_str[MAX_REG_STR_LEN];	

	if (regNum >= NUM_REGS) 
	{
		logExeCStyleSimplified("ERROR(%d): %s: invalid regNum %d\n\n", 
			csvEntryNum, __func__, regNum);
		return;
	}

	vsnprintf(reg_str, MAX_REG_STR_LEN, format, arg);

	uint32_t stack_depth = regStackDepths[regNum];
	memcpy((void*)descStrs->reg[regNum][stack_depth], reg_str, 
		MAX_REG_STR_LEN);

	logExeSetCondDescStrOnly(descStrs, cond, reg_str);
}

/**
 * Return the string describing the specified register.
 * 
 * \param descStr[in] Pointer to DescStrs to be updated (i.e. may pertain to
 *	value description strings or comment description strings).
 * \param regNum Register number to return descriptive string for.
 *
 * \return Pointer to string descibed above.
 */
static const char* logExeGetRegDescStr(const struct DescStrs* descStrs, 
	uint32_t regNum) 
{
	if (regNum >= NUM_REGS) 
	{
		logExeCStyleSimplified("ERROR(%d): %s: invalid regNum %d\n\n", 
			csvEntryNum, __func__, regNum);
		return "ERROR: invalid regNum";
	}

	uint32_t stack_depth = regStackDepths[regNum];

	return descStrs->reg[regNum][stack_depth];
}

/**
 * Set state of the value stored in the register. This is used by simplified
 *  C style logging to know whether intermediate actions can be reduced to
 *  current values, or actions should be left in long form.
 *
 * \param regNum Number of register for info to be updated.
 * \param isConstVal TRUE indicates value is constant-based and reductions
 *  can be stored. FALSE indicates long form should be stored.
 *
 * \return None.
 */
static void logExeSetRegHasConstVal(uint32_t regNum, int isConstVal)
{
	if (regNum >= NUM_REGS) 
	{
		logExeCStyleSimplified("ERROR(%d): %s: invalid regNum %d\n\n", 
			csvEntryNum, __func__, regNum);
	}

	uint32_t stack_depth = regStackDepths[regNum];

	hasConstVals.reg[regNum][stack_depth] = isConstVal;
}

/**
 * Like logExeSetRegHasConstVal() but for conditionals.
 *
 * \param cond Defines which conditional bits are affected and which
 *  conditional consts states should be updated. Use APSR_* macros.
 * \param isConstVal TRUE indicates value is constant-based and reductions
 *  can be stored. FALSE indicates long form should be stored.
 *
 * \return None.
 */
static void logExeSetCondHasConstVal(uint32_t cond, int isConstVal)
{
	if (cond & APSR_N)
	{
		hasConstVals.apsr_n = isConstVal;
	}
	if (cond & APSR_Z)
	{
		hasConstVals.apsr_z = isConstVal;
	}
	if (cond & APSR_C)
	{
		hasConstVals.apsr_c = isConstVal;
	}
	if (cond & APSR_V)
	{
		hasConstVals.apsr_v = isConstVal;
	}
}

/**
 * Update the log string for the given register. Associated conditional strings
 *  are also updated if need be.
 *
 * \param regNum 0 based register number that defines which string is to be
 *  updated. 
 * \param cond Defines which conditional bits are affected and which
 *  conditional strings should be updated. Use APSR_* macros.
 * \param[in] format Human readable description of entry.
 * \param arg A value identifying a variable arguments list initialized with va_start.
 * 
 * \return None.
 */
void logExeSetRegValStr(uint32_t regNum, uint32_t cond, int isConstVal, 
	const char* format, ...) 
{
	va_list args;
	va_start(args, format);
	logExeSetRegDescStr(&valStrs, regNum, cond, format, args);
	va_end(args);

	logExeSetRegHasConstVal(regNum, isConstVal);

	logExeSetCondHasConstVal(cond, isConstVal);
}

/**
 * Return the string describing how the specified register got its value (i.e.
 *  from a memory read).
 * 
 * \param regNum Register number to return descriptive string for.
 *
 * \return Pointer to string descibed above.
 */
const char* logExeGetRegValStr(uint32_t regNum) 
{
	return logExeGetRegDescStr(&valStrs, regNum);
}

/**
 * See logExeSetRegValStr(), except description string relates to comments.
 */
void logExeSetRegCmtStr(uint32_t regNum, uint32_t cond, const char* format, ...) 
{
	va_list args;
	va_start(args, format);
	logExeSetRegDescStr(&cmtStrs, regNum, cond, format, args);
	va_end(args);
}

/**
 * See logExeGetRegValStr(), except description string relates to comments.
 */
const char* logExeGetRegCmtStr(uint32_t regNum) 
{
	return logExeGetRegDescStr(&cmtStrs, regNum);
}

/**
 * "Push" register description strings that are updated by logExeSetReg*Str() 
 *  onto stack so that previous value can be recalled with call to 
 *  logExePopRegStrs().
 *
 * \parma regNum The 0-based register number whose description string is to 
 *  be push onto the stack.
 *
 * \return None.
 */
void logExePushRegStrs(uint32_t regNum) 
{
	if (regNum >= NUM_REGS) 
	{
		logExeCStyleSimplified("ERROR(%d): %s: invalid regNum %d\n\n", 
			csvEntryNum, __func__, regNum);
		return;
	}

	uint32_t stack_depth = regStackDepths[regNum];
	if (stack_depth >= MAX_REG_STR_STACK_SIZE-1)
	{
		logExeCStyleSimplified("ERROR(%d): %s: Attempt to go above max stack depth for reg %d\n\n", 
			csvEntryNum, __func__, regNum);
		return;
	}
	regStackDepths[regNum] = stack_depth+1;
}

/**
 * Inverse of logExePushRegStrs(). This function recalls previously saved 
 *  register description string.
 *
 * \parma regNum The 0-based register number whose description string is to 
 *  be push onto the stack.
 *
 * \return None.
 */
void logExePopRegStrs(uint32_t regNum) 
{
	if (regNum >= NUM_REGS) 
	{
		logExeCStyleSimplified("ERROR(%d): %s: invalid regNum %d\n\n", 
			csvEntryNum, __func__, regNum);
		return;
	}

	uint32_t stack_depth = regStackDepths[regNum];
	if (!stack_depth)
	{
		logExeCStyleSimplified("ERROR(%d): %s: Attempt to go below min stack depth for reg %d\n\n", 
			csvEntryNum, __func__, regNum);
		return;
	}
	regStackDepths[regNum] = stack_depth-1;
}

/**
 * Similar to logExeSetRegValStr, except only conditional strings are updated.
 *
 * \param cond Defines which conditional bits are affected and which
 *  conditional strings should be updated. Use APSR_* macros.
 * \param[in] format Human readable description of entry.
 * \param arg A value identifying a variable arguments list initialized with va_start.
 * 
 * \return None.
 */
void logExeSetCondValStr(uint32_t cond, int isConstVal, const char* format, ...) {
	char cond_str[MAX_REG_STR_LEN];	
	va_list args;

	va_start(args, format);
	vsnprintf(cond_str, MAX_REG_STR_LEN, format, args);
	va_end(args);

	logExeSetCondDescStrOnly(&valStrs, cond, cond_str);

	logExeSetCondHasConstVal(cond, isConstVal);
}

/**
 * Return the string relating to the actions that last caused the specified 
 *  conditional bits to be set.
 *
 * \param descStr[in] Pointer to DescStrs to be updated (i.e. may pertain to
 *	value description strings or comment description strings).
 * \param cond Defines which conditional(s) related string(s) to return.
 *
 * \return String as described above. 
 */
static const char* logExeGetCondDescStr(const struct DescStrs* descStrs, 
	uint32_t cond) 
{
	switch (cond) 
	{
	case APSR_Z | APSR_N | APSR_V:
		if (strncmp(descStrs->apsr_n, descStrs->apsr_z, MAX_REG_STR_LEN) ||
			strncmp(descStrs->apsr_n, descStrs->apsr_v, MAX_REG_STR_LEN)) 
		{
			logExeCStyleSimplified("ERROR(%d): %s: APSR_Z | APSR_N | APSR_V strings do not match\n\n", 
				csvEntryNum, __func__);
			return "ERROR: logExeGetCondDescStr(): APSR_Z | APSR_N | APSR_V string do not match";
		}
		return descStrs->apsr_n;

	case APSR_N | APSR_V:
		if (strncmp(descStrs->apsr_n, descStrs->apsr_v, MAX_REG_STR_LEN)) 
		{
			logExeCStyleSimplified("ERROR(%d): %s: APSR_N | APSR_V strings do not match\n\n", 
				csvEntryNum, __func__);
			return "ERROR: logExeGetCondDescStr(): APSR_N | APSR_V string do not match";
		}
		return descStrs->apsr_n;

	case APSR_C | APSR_Z:
		if (strncmp(descStrs->apsr_z, descStrs->apsr_c, MAX_REG_STR_LEN)) 
		{
			logExeCStyleSimplified("ERROR(%d): %s: APSR_C | APSR_Z strings do not match\n\n", 
				csvEntryNum, __func__);
			return "ERROR: logExeGetCondDescStr(): APSR_C | APSR_Z string do not match";
		}
		return descStrs->apsr_z;

	case APSR_N:
		return descStrs->apsr_n;

	case APSR_Z:
		return descStrs->apsr_z;

	case APSR_C:
		return descStrs->apsr_c;

	case APSR_V:
		return descStrs->apsr_v;
	}

	return "ERROR: logExeGetCondDescStr(): Invalid cond combo";
}

/**
 * Return the string descrbing the actions that last caused the specified 
 *  conditional bits to be set.
 *
 * \param cond Defines which conditional related string(s) to return.
 *
 * \return String as described above. 
 */
const char* logExeGetCondValStr(uint32_t cond)
{
	return logExeGetCondDescStr(&valStrs, cond);
}

/**
 * See logExeSetCondValStr(), except description string relates to comments.
 */
void logExeSetCondCmtStr(uint32_t cond, const char* format, ...)
{
	char cond_str[MAX_REG_STR_LEN];	
	va_list args;

	va_start(args, format);
	vsnprintf(cond_str, MAX_REG_STR_LEN, format, args);
	va_end(args);

	logExeSetCondDescStrOnly(&cmtStrs, cond, cond_str);
}

/**
 * See logExeGetCondValStr(), exception description string relates to comments.
 */
const char* logExeGetCondCmtStr(uint32_t cond) {
	return logExeGetCondDescStr(&cmtStrs, cond);
}

/**
 * Add entry to simplified exeLogCFile. This is similar to logExeCStyleVerbose(), except
 *  it is written to a different file where the goal is to reduce all register
 *  read/writes to memory accesses or conditional checks.
 *
 * \param[in] format Human readable description of instruction.
 * \param ... Additional argument like for printf().
 *
 * \return None.
 */
void logExeCStyleSimplified(const char* format, ...) {
	static needs_indent = 1;
	va_list args;

	// Check if logging was enabled
	if (!exeLogCsvFile)
		return;

	if (needs_indent) 
	{
		for (int cnt = 0; cnt < exeLogCNumTabs; cnt++) 
		{
			fprintf(exeLogCSimpliedFile, "\t");
		}

		needs_indent = 0;
	}

	va_start(args, format);
	vfprintf(exeLogCSimpliedFile, format, args);
	va_end(args);

	if (format[strlen(format)-1] == '\n') 
	{
		needs_indent = 1;
	}

	fflush(exeLogCSimpliedFile);
}

/**
 * \param regNum Number of register for info to be updated.
 *
 * \return TRUE if register value was computed via constant(s) only. 
 *  FALSE otherwise and long form of instructions leading to current value
 *  should be stored for simplified logging.
 */
int logExeGetRegHasConstVal(uint32_t regNum)
{
	if (regNum >= NUM_REGS) 
	{
		logExeCStyleSimplified("ERROR(%d): %s: invalid regNum %d\n\n", 
			csvEntryNum, __func__, regNum);
		return FALSE;
	}

	uint32_t stack_depth = regStackDepths[regNum];

	return hasConstVals.reg[regNum][stack_depth];
}

/**
 * Like logExeGetRegHasConstVal() but for conditionals.
 *
 * \param cond Defines which conditional bits are affected and which
 *  conditional consts states should be updated. Use APSR_* macros.
 *
 * \return TRUE if instructions related to given conditional are constant
 *  based. False otherwise.
 */
int logExeGetCondHasConstVal(uint32_t cond)
{
	switch (cond) 
	{
	case APSR_Z | APSR_N | APSR_V:
		if (hasConstVals.apsr_n != hasConstVals.apsr_z ||
			hasConstVals.apsr_n != hasConstVals.apsr_v)
		{
			logExeCStyleSimplified("ERROR(%d): %s: APSR_Z | APSR_N | APSR_V match issue\n\n", 
				csvEntryNum, __func__);
			return FALSE;
		}
		return hasConstVals.apsr_n;

	case APSR_N | APSR_V:
		if (hasConstVals.apsr_n !=  hasConstVals.apsr_v)
		{
			logExeCStyleSimplified("ERROR(%d): %s: APSR_N | APSR_V match issue\n\n", 
				csvEntryNum, __func__);
			return FALSE;
		}
		return hasConstVals.apsr_n;

	case APSR_C | APSR_Z:
		if (hasConstVals.apsr_z !=  hasConstVals.apsr_c)
		{
			logExeCStyleSimplified("ERROR(%d): %s: APSR_C | APSR_Z match issue\n\n", 
				csvEntryNum, __func__);
			return FALSE;
		}
		return hasConstVals.apsr_n;

	case APSR_N:
		return hasConstVals.apsr_n;

	case APSR_Z:
		return hasConstVals.apsr_z;

	case APSR_C:
		return hasConstVals.apsr_c;

	case APSR_V:
		return hasConstVals.apsr_v;
	}

	logExeCStyleSimplified("ERROR(%d): %s: invalid case\n\n", 
		csvEntryNum, __func__);
	return FALSE;
}
