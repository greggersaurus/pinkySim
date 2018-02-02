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
} MemInfoEntry;

// Used to (potentially) give additional details on memory sections
static const struct MemInfoEntry* memInfo = NULL;
// Number of entires in memInfo
static uint32_t memInfoSize = 0;

// Memory sections specific to NXP LPC11U37 Chip
//  Not to be accessed directly, but rather an option that memInfo can be set to.
static const struct MemInfoEntry LPC11U37_MEM_INFO[] = 
{
	{0x00000000, 0x00020000-1, "128 kB on-chip flash"}, // --flash 0 131072
	{0x10000000, 0x10002000-1, "8 kB SRAM0"}, // --ram 268435456 8192
	{0x1FFF0000, 0x1FFF4000-1, "16 kB boot ROM"}, // --flash 536805376 16384
	{0x20000000, 0x20000800-1, "2 kB SRAM1"}, // --ram 536870912 2048
	{0x20004000, 0x20004800-1, "2 kB USB SRAM"}, // --ram 536887296 2048

	//{0x40000000, 0x40004000-1, "I2C-bus"}, // --ram 1073741824 16384
	{0x40000000, 0x40000004-1, "I2C: CONSET"},
	{0x40000004, 0x40000008-1, "I2C: STAT"},
	{0x40000008, 0x4000000C-1, "I2C: DAT"},
	{0x4000000C, 0x40000010-1, "I2C: ADR0"},
	{0x40000010, 0x40000014-1, "I2C: SCLH"},
	{0x40000014, 0x40000018-1, "I2C: SCLL"},
	{0x40000018, 0x4000001C-1, "I2C: CONCLR"},
	{0x4000001C, 0x40000020-1, "I2C: MCCTRL"},
	{0x40000020, 0x40000024-1, "I2C: ADR1"},
	{0x40000024, 0x40000028-1, "I2C: ADR2"},
	{0x40000028, 0x4000002C-1, "I2C: ADR3"},
	{0x4000002C, 0x40000030-1, "I2C: DATA_BUFFER"},
	{0x40000030, 0x40000034-1, "I2C: MASK0"},
	{0x40000034, 0x40000038-1, "I2C: MASK1"},
	{0x40000038, 0x4000003C-1, "I2C: MASK2"},
	{0x4000003C, 0x40000040-1, "I2C: MASK3"},

	//{0x40004000, 0x40008000-1, "WWDT"}, // --ram 1073758208 16384
	{0x40004000, 0x40004004-1, "WWDT: MOD"},
	{0x40004004, 0x40004008-1, "WWDT: TC"},
	{0x40004008, 0x4000400C-1, "WWDT: FEED"},
	{0x4000400C, 0x40004010-1, "WWDT: TV"},
	{0x40004010, 0x40004014-1, "WWDT: CLKSEL"},
	{0x40004014, 0x40004018-1, "WWDT: WARNINT"},
	{0x40004018, 0x4000401C-1, "WWDT: WINDOW"},

	//{0x40008000, 0x4000C000-1, "USART/SMART CARD"}, // --ram 1073774592 16384
	{0x40008000, 0x40008004-1, "USART: RBR/THR or DLL"},
	{0x40008004, 0x40008008-1, "USART: DLM or IER"},
	{0x40008008, 0x4000800C-1, "USART: IIR or FCR"},
	{0x4000800C, 0x40008010-1, "USART: LCR"},
	{0x40008010, 0x40008014-1, "USART: MCR"},
	{0x40008014, 0x40008018-1, "USART: LSR"},
	{0x40008018, 0x4000801C-1, "USART: MSR"},
	{0x4000801C, 0x40008020-1, "USART: SCR"},
	{0x40008020, 0x40008024-1, "USART: ACR"},
	{0x40008024, 0x40008028-1, "USART: ICR"},
	{0x40008028, 0x4000802C-1, "USART: FDR"},
	{0x4000802C, 0x40008030-1, "USART: OSR"},
	{0x40008030, 0x40008034-1, "USART: TER"},
	{0x40008040, 0x40008044-1, "USART: HDEN"},
	{0x40008048, 0x4000804C-1, "USART: SCICTRL"},
	{0x4000804C, 0x40008050-1, "USART: RS485CTRL"},
	{0x40008050, 0x40008054-1, "USART: RS485ADRMATCH"},
	{0x40008054, 0x40008058-1, "USART: RS485DLY"},
	{0x40008058, 0x4000805C-1, "USART: SYNCCTRL"},

	//{0x4000C000, 0x40010000-1, "16-bit counter/timer 0"}, // --ram 1073790976 16384
	{0x4000C000, 0x4000C004-1, "CT16B0: IR"},
	{0x4000C004, 0x4000C008-1, "CT16B0: TCR"},
	{0x4000C008, 0x4000C00C-1, "CT16B0: TC"},
	{0x4000C00C, 0x4000C010-1, "CT16B0: PR"},
	{0x4000C010, 0x4000C014-1, "CT16B0: PC"},
	{0x4000C014, 0x4000C018-1, "CT16B0: MCR"},
	{0x4000C018, 0x4000C01C-1, "CT16B0: MR0"},
	{0x4000C01C, 0x4000C020-1, "CT16B0: MR1"},
	{0x4000C020, 0x4000C024-1, "CT16B0: MR2"},
	{0x4000C024, 0x4000C028-1, "CT16B0: MR3"},
	{0x4000C028, 0x4000C02C-1, "CT16B0: CCR"},
	{0x4000C02C, 0x4000C030-1, "CT16B0: CR0"},
	{0x4000C034, 0x4000C038-1, "CT16B0: CR1"},
	{0x4000C03C, 0x4000C040-1, "CT16B0: EMR"},
	{0x4000C070, 0x4000C074-1, "CT16B0: CTCR"},
	{0x4000C074, 0x4000C078-1, "CT16B0: PWMC"},

	//{0x40010000, 0x40014000-1, "16-bit counter/timer 1"}, // --ram 1073807360 16384
	{0x40010000, 0x40010004-1, "CT16B1: IR"},
	{0x40010004, 0x40010008-1, "CT16B1: TCR"},
	{0x40010008, 0x4001000C-1, "CT16B1: TC"},
	{0x4001000C, 0x40010010-1, "CT16B1: PR"},
	{0x40010010, 0x40010014-1, "CT16B1: PC"},
	{0x40010014, 0x40010018-1, "CT16B1: MCR"},
	{0x40010018, 0x4001001C-1, "CT16B1: MR0"},
	{0x4001001C, 0x40010020-1, "CT16B1: MR1"},
	{0x40010020, 0x40010024-1, "CT16B1: MR2"},
	{0x40010024, 0x40010028-1, "CT16B1: MR3"},
	{0x40010028, 0x4001002C-1, "CT16B1: CCR"},
	{0x4001002C, 0x40010030-1, "CT16B1: CR0"},
	{0x40010034, 0x40010038-1, "CT16B1: CR1"},
	{0x4001003C, 0x40010040-1, "CT16B1: EMR"},
	{0x40010070, 0x40010074-1, "CT16B1: CTCR"},
	{0x40010074, 0x40010078-1, "CT16B1: PWMC"},

	//{0x40014000, 0x40018000-1, "32-bit counter/timer 0"}, // --ram 1073823744 16384
	{0x40014000, 0x40014004-1, "CT32B0: IR"},
	{0x40014004, 0x40014008-1, "CT32B0: TCR"},
	{0x40014008, 0x4001400C-1, "CT32B0: TC"},
	{0x4001400C, 0x40014010-1, "CT32B0: PR"},
	{0x40014010, 0x40014014-1, "CT32B0: PC"},
	{0x40014014, 0x40014018-1, "CT32B0: MCR"},
	{0x40014018, 0x4001401C-1, "CT32B0: MR0"},
	{0x4001401C, 0x40014020-1, "CT32B0: MR1"},
	{0x40014020, 0x40014024-1, "CT32B0: MR2"},
	{0x40014024, 0x40014028-1, "CT32B0: MR3"},
	{0x40014028, 0x4001402C-1, "CT32B0: CCR"},
	{0x4001402C, 0x40014030-1, "CT32B0: CR0"},
	{0x40014034, 0x40014038-1, "CT32B0: CR1"},
	{0x4001403C, 0x40014040-1, "CT32B0: EMR"},
	{0x40014070, 0x40014074-1, "CT32B0: CTCR"},
	{0x40014074, 0x40014078-1, "CT32B0: PWMC"},

	//{0x40018000, 0x4001C000-1, "32-bit counter/timer 1"}, // --ram 1073840128 16384
	{0x40018000, 0x40018004-1, "CT32B1: IR"},
	{0x40018004, 0x40018008-1, "CT32B1: TCR"},
	{0x40018008, 0x4001800C-1, "CT32B1: TC"},
	{0x4001800C, 0x40018010-1, "CT32B1: PR"},
	{0x40018010, 0x40018014-1, "CT32B1: PC"},
	{0x40018014, 0x40018018-1, "CT32B1: MCR"},
	{0x40018018, 0x4001801C-1, "CT32B1: MR0"},
	{0x4001801C, 0x40018020-1, "CT32B1: MR1"},
	{0x40018020, 0x40018024-1, "CT32B1: MR2"},
	{0x40018024, 0x40018028-1, "CT32B1: MR3"},
	{0x40018028, 0x4001802C-1, "CT32B1: CCR"},
	{0x4001802C, 0x40018030-1, "CT32B1: CR0"},
	{0x40018034, 0x40018038-1, "CT32B1: CR1"},
	{0x4001803C, 0x40018040-1, "CT32B1: EMR"},
	{0x40018070, 0x40018074-1, "CT32B1: CTCR"},
	{0x40018074, 0x40018078-1, "CT32B1: PWMC"},

	//{0x4001C000, 0x40020000-1, "ADC"}, // --ram 1073856512 16384
	{0x4001C000, 0x4001C004-1, "ADC: CR"},
	{0x4001C004, 0x4001C008-1, "ADC: GDR"},
	{0x4001C00C, 0x4001C010-1, "ADC: INTEN"},
	{0x4001C010, 0x4001C014-1, "ADC: DR0"},
	{0x4001C014, 0x4001C018-1, "ADC: DR1"},
	{0x4001C018, 0x4001C01C-1, "ADC: DR2"},
	{0x4001C01C, 0x4001C020-1, "ADC: DR3"},
	{0x4001C020, 0x4001C024-1, "ADC: DR4"},
	{0x4001C024, 0x4001C028-1, "ADC: DR5"},
	{0x4001C028, 0x4001C02C-1, "ADC: DR6"},
	{0x4001C02C, 0x4001C030-1, "ADC: DR7"},
	{0x4001C030, 0x4001C034-1, "ADC: STAT"},

	//{0x40038000, 0x4003C000-1, "PMU"}, // --ram 1073971200 16384
	{0x40038000, 0x40038004-1, "PMU: PCON"},
	{0x40038004, 0x40038008-1, "PMU: GPREG0"},
	{0x40038008, 0x4003800C-1, "PMU: GPREG1"},
	{0x4003800C, 0x40038010-1, "PMU: GPREG4"},
	{0x40038010, 0x40038014-1, "PMU: GPREG3"},
	{0x40038014, 0x40038018-1, "PMU: GPREG4"},

	//{0x4003C000, 0x40040000-1, "flash/EEPROM controller"}, // --ram 1073987584 16384
	{0x4003C010, 0x4003C014-1, "flash/EEPROM controller: FLASHCFG"},	

	//{0x40040000, 0x40044000-1, "SSPO"}, // --ram 1074003968 16384
	{0x40040000, 0x40040004-1, "SSP/SPI0: Ctrl Reg 0"},
	{0x40040004, 0x40040008-1, "SSP/SPI0: Ctrl Reg 1"},
	{0x40040008, 0x4004000C-1, "SSP/SPI0: Data Reg"},
	{0x4004000C, 0x40040010-1, "SSP/SPI0: Status Reg"},
	{0x40040010, 0x40040014-1, "SSP/SPI0: Clk Prescale Reg"},
	{0x40040014, 0x40040018-1, "SSP/SPI0: IMSC"},
	{0x40040018, 0x4004001C-1, "SSP/SPI0: RIS"},
	{0x4004001C, 0x40040020-1, "SSP/SPI0: MIS"},
	{0x40040020, 0x40040024-1, "SSP/SPI0: ICR"},

	//{0x40044000, 0x40048000-1, "IOCON"}, // --ram 1074020352 16384
	{0x40044000, 0x40044004-1, "IOCON: RESET_PIO0_0"},
	{0x40044004, 0x40044008-1, "IOCON: PIO0_1"},
	{0x40044008, 0x4004400C-1, "IOCON: PIO0_2"},
	{0x4004400C, 0x40044010-1, "IOCON: PIO0_3"},
	{0x40044010, 0x40044014-1, "IOCON: PIO0_4"},
	{0x40044014, 0x40044018-1, "IOCON: PIO0_5"},
	{0x40044018, 0x4004401C-1, "IOCON: PIO0_6"},
	{0x4004401C, 0x40044020-1, "IOCON: PIO0_7"},
	{0x40044020, 0x40044024-1, "IOCON: PIO0_8"},
	{0x40044024, 0x40044028-1, "IOCON: PIO0_9"},
	{0x40044028, 0x4004402C-1, "IOCON: SWCLK_PIO0_10"},
	{0x4004402C, 0x40044030-1, "IOCON: TDI_PIO0_11"},
	{0x40044030, 0x40044034-1, "IOCON: TMS_PIO0_12"},
	{0x40044034, 0x40044038-1, "IOCON: TDO_PIO0_13"},
	{0x40044038, 0x4004403C-1, "IOCON: TRST_PIO0_14"},
	{0x4004403C, 0x40044040-1, "IOCON: SWDIO_PIO0_15"},
	{0x40044040, 0x40044044-1, "IOCON: PIO0_16"},
	{0x40044044, 0x40044048-1, "IOCON: PIO0_17"},
	{0x40044048, 0x4004404C-1, "IOCON: PIO0_18"},
	{0x4004404C, 0x40044050-1, "IOCON: PIO0_19"},
	{0x40044050, 0x40044054-1, "IOCON: PIO0_20"},
	{0x40044054, 0x40044058-1, "IOCON: PIO0_21"},
	{0x40044058, 0x4004405C-1, "IOCON: PIO0_22"},
	{0x4004405C, 0x40044060-1, "IOCON: PIO0_23"},
	{0x40044060, 0x40044064-1, "IOCON: PIO1_0"},
	{0x40044064, 0x40044068-1, "IOCON: PIO1_1"},
	{0x40044068, 0x4004406C-1, "IOCON: PIO1_2"},
	{0x4004406C, 0x40044070-1, "IOCON: PIO1_3"},
	{0x40044070, 0x40044074-1, "IOCON: PIO1_4"},
	{0x40044074, 0x40044078-1, "IOCON: PIO1_5"},
	{0x40044078, 0x4004407C-1, "IOCON: PIO1_6"},
	{0x4004407C, 0x40044080-1, "IOCON: PIO1_7"},
	{0x40044080, 0x40044084-1, "IOCON: PIO1_8"},
	{0x40044084, 0x40044088-1, "IOCON: PIO1_9"},
	{0x40044088, 0x4004408C-1, "IOCON: PIO1_10"},
	{0x4004408C, 0x40044090-1, "IOCON: PIO1_11"},
	{0x40044090, 0x40044094-1, "IOCON: PIO1_12"},
	{0x40044094, 0x40044098-1, "IOCON: PIO1_13"},
	{0x40044098, 0x4004409C-1, "IOCON: PIO1_14"},
	{0x4004409C, 0x400440A0-1, "IOCON: PIO1_15"},
	{0x400440A0, 0x400440A4-1, "IOCON: PIO1_16"},
	{0x400440A4, 0x400440A8-1, "IOCON: PIO1_17"},
	{0x400440A8, 0x400440AC-1, "IOCON: PIO1_18"},
	{0x400440AC, 0x400440B0-1, "IOCON: PIO1_19"},
	{0x400440B0, 0x400440B4-1, "IOCON: PIO1_20"},
	{0x400440B4, 0x400440B8-1, "IOCON: PIO1_21"},
	{0x400440B8, 0x400440BC-1, "IOCON: PIO1_22"},
	{0x400440BC, 0x400440C0-1, "IOCON: PIO1_23"},
	{0x400440C0, 0x400440C4-1, "IOCON: PIO1_24"},
	{0x400440C4, 0x400440C8-1, "IOCON: PIO1_25"},
	{0x400440C8, 0x400440CC-1, "IOCON: PIO1_26"},
	{0x400440CC, 0x400440D0-1, "IOCON: PIO1_27"},
	{0x400440D0, 0x400440D4-1, "IOCON: PIO1_28"},
	{0x400440D4, 0x400440D8-1, "IOCON: PIO1_29"},
	{0x400440DC, 0x400440E0-1, "IOCON: PIO1_31"},

	//{0x40048000, 0x4004C000-1, "system control"}, // --ram 1074036736 16384
	{0x40048000, 0x40048004-1, "system control: SYSMEMREMAP"},
	{0x40048004, 0x40048008-1, "system control: PRESETCTRL"},
	{0x40048008, 0x4004800C-1, "system control: SYSPLLCTRL"},
	{0x4004800C, 0x40048010-1, "system control: SYSPLLSTAT"},
	{0x40048010, 0x40048014-1, "system control: USBPLLCTRL"},
	{0x40048014, 0x40048018-1, "system control: USBPLLSTAT"},
	{0x40048020, 0x40048024-1, "system control: SYSOSCCTRL"},
	{0x40048024, 0x40048028-1, "system control: WDTOSCCTRL"},
	{0x40048028, 0x4004802C-1, "system control: IRCCTRL"},
	{0x40048030, 0x40048034-1, "system control: SYSRSTSTAT"},
	{0x40048040, 0x40048044-1, "system control: SYSPLLCLKSEL"},
	{0x40048044, 0x40048048-1, "system control: SYSPLLCLKUEN"},
	{0x40048070, 0x40048074-1, "system control: MAINCLKSEL"},
	{0x40048074, 0x40048078-1, "system control: MAINCLKUEN"},
	{0x40048078, 0x4004807C-1, "system control: SYSAHBCLKDIV"},
	{0x40048080, 0x40048084-1, "system control: SYSAHBCLKCTRL"},
	{0x40048094, 0x40048098-1, "system control: SSP0CLKDIV"},
	{0x40048098, 0x4004809C-1, "system control: UARTCLKDIV"},
	{0x4004809C, 0x400480A0-1, "system control: SSP1CLKDIV"},
	{0x400480C0, 0x400480C4-1, "system control: USBCLKSEL"},
	{0x400480C4, 0x400480C8-1, "system control: USBCLKUEN"},
	{0x400480C8, 0x400480CC-1, "system control: USBCLKDIV"},
	{0x400480E0, 0x400480E4-1, "system control: CLKOUTSEL"},
	{0x400480E4, 0x400480E8-1, "system control: CLKOUTUEN"},
	{0x400480E8, 0x400480EC-1, "system control: CLKOUTDIV"},
	{0x40048100, 0x40048104-1, "system control: PIOPORCAP0"},
	{0x40048104, 0x40048108-1, "system control: PIOPORCAP1"},
	{0x40048150, 0x40048154-1, "system control: BODCTRL"},
	{0x40048154, 0x40048158-1, "system control: SYSTCKCAL"},
	{0x40048170, 0x40048174-1, "system control: IRQLATENCY"},
	{0x40048174, 0x40048178-1, "system control: NMISRC"},
	{0x40048178, 0x4004817C-1, "system control: PINTSEL0"},
	{0x4004817C, 0x40048180-1, "system control: PINTSEL1"},
	{0x40048180, 0x40048184-1, "system control: PINTSEL2"},
	{0x40048184, 0x40048188-1, "system control: PINTSEL3"},
	{0x40048188, 0x4004818C-1, "system control: PINTSEL4"},
	{0x4004818C, 0x40048190-1, "system control: PINTSEL5"},
	{0x40048190, 0x40048194-1, "system control: PINTSEL6"},
	{0x40048194, 0x40048198-1, "system control: PINTSEL7"},
	{0x40048198, 0x4004819C-1, "system control: USBCLKCTRL"},
	{0x4004819C, 0x400481A0-1, "system control: USBCLKST"},
	{0x40048204, 0x40048208-1, "system control: STARTERP0"},
	{0x40048214, 0x40048218-1, "system control: STARTERP1"},
	{0x40048230, 0x40048234-1, "system control: PDSLEEPCFG"},
	{0x40048234, 0x40048238-1, "system control: PDWAKECFG"},
	{0x40048238, 0x4004823C-1, "system control: PDRUNCFG"},
	{0x400483F4, 0x400483F8-1, "system control: DEVICE_ID"},

	//{0x4004C000, 0x40050000-1, "GPIO interrupts"}, // --ram 1074053120 16384
	{0x4004C000, 0x4004C004-1, "GPIO interrupts: ISEL"},
	{0x4004C004, 0x4004C008-1, "GPIO interrupts: IENR"},
	{0x4004C008, 0x4004C00C-1, "GPIO interrupts: SIENR"},
	{0x4004C00C, 0x4004C010-1, "GPIO interrupts: CIENR"},
	{0x4004C010, 0x4004C014-1, "GPIO interrupts: IENF"},
	{0x4004C014, 0x4004C018-1, "GPIO interrupts: SIENF"},
	{0x4004C018, 0x4004C01C-1, "GPIO interrupts: CIENF"},
	{0x4004C01C, 0x4004C020-1, "GPIO interrupts: RISE"},
	{0x4004C020, 0x4004C024-1, "GPIO interrupts: FALL"},
	{0x4004C024, 0x4004C028-1, "GPIO interrupts: IST"},

	//{0x40058000, 0x4005C000-1, "SSP1"}, // --ram 1074102272 16384
	{0x40058000, 0x40058004-1, "SSP/SPI1: Ctrl Reg 0"},
	{0x40058004, 0x40058008-1, "SSP/SPI1: Ctrl Reg 1"},
	{0x40058008, 0x4005800C-1, "SSP/SPI1: Data Reg"},
	{0x4005800C, 0x40058010-1, "SSP/SPI1: Status Reg"},
	{0x40058010, 0x40058014-1, "SSP/SPI1: Clk Prescale Reg"},
	{0x40058014, 0x40058018-1, "SSP/SPI1: IMSC"},
	{0x40058018, 0x4005801C-1, "SSP/SPI1: RIS"},
	{0x4005801C, 0x40058020-1, "SSP/SPI1: MIS"},
	{0x40058020, 0x40058024-1, "SSP/SPI1: ICR"},

	//{0x4005C000, 0x40060000-1, "GPIO GROUP0 INT"}, // --ram 1074118656 16384
	{0x4005C000, 0x4005C004-1, "GPIO GROUP0 INT: CTRL"},
	{0x4005C020, 0x4005C024-1, "GPIO GROUP0 INT: PORT_POL0"},
	{0x4005C024, 0x4005C028-1, "GPIO GROUP0 INT: PORT_POL1"},
	{0x4005C028, 0x4005C02C-1, "GPIO GROUP0 INT: PORT_POL2"},
	{0x4005C040, 0x4005C044-1, "GPIO GROUP0 INT: PORT_ENA0"},
	{0x4005C044, 0x4005C04C-1, "GPIO GROUP0 INT: PORT_ENA1"},

	//{0x40060000, 0x40064000-1, "GPIO GROUP1 INT"}, // --ram 1074135040 16384
	{0x40060000, 0x40060004-1, "GPIO GROUP1 INT: CTRL"},
	{0x40060020, 0x40060024-1, "GPIO GROUP1 INT: PORT_POL0"},
	{0x40060024, 0x40060028-1, "GPIO GROUP1 INT: PORT_POL1"},
	{0x40060028, 0x4006002C-1, "GPIO GROUP1 INT: PORT_POL2"},
	{0x40060040, 0x40060044-1, "GPIO GROUP1 INT: PORT_ENA0"},
	{0x40060044, 0x4006004C-1, "GPIO GROUP1 INT: PORT_ENA1"},

	//{0x40080000, 0x40084000-1, "USB"}, // --ram 1074266112 16384
	{0x40080000, 0x40080004-1, "USB: DEVCMDSTAT"},
	{0x40080004, 0x40080008-1, "USB: INFO"},
	{0x40080008, 0x4008000C-1, "USB: EPLISTSTART"},
	{0x4008000C, 0x40080010-1, "USB: DATABUFFSTART"},
	{0x40080010, 0x40080014-1, "USB: LPM"},
	{0x40080014, 0x40080018-1, "USB: EPSKIP"},
	{0x40080018, 0x4008001C-1, "USB: EPINUSE"},
	{0x4008001C, 0x40080020-1, "USB: EPBUFCFG"},
	{0x40080020, 0x40080024-1, "USB: INTSTAT"},
	{0x40080024, 0x40080028-1, "USB: INTEN"},
	{0x40080028, 0x4008002C-1, "USB: INTSETSTAT"},
	{0x4008002C, 0x40080030-1, "USB: INTROUTING"},
	{0x40080030, 0x40080034-1, "USB: EPTOGGLE"},

	//{0x50000000, 0x50004000-1, "GPIO"}, // --ram 1342177280 16484
	{0x50000000, 0x50000000, "GPIO: P0_0 PBYTE"},
	{0x50000001, 0x50000001, "GPIO: P0_1 PBYTE"},
	{0x50000002, 0x50000002, "GPIO: P0_2 PBYTE"},
	{0x50000003, 0x50000003, "GPIO: P0_3 PBYTE"},
	{0x50000004, 0x50000004, "GPIO: P0_4 PBYTE"},
	{0x50000005, 0x50000005, "GPIO: P0_5 PBYTE"},
	{0x50000006, 0x50000006, "GPIO: P0_6 PBYTE"},
	{0x50000007, 0x50000007, "GPIO: P0_7 PBYTE"},
	{0x50000008, 0x50000008, "GPIO: P0_8 PBYTE"},
	{0x50000009, 0x50000009, "GPIO: P0_9 PBYTE"},
	{0x5000000A, 0x5000000A, "GPIO: P0_10 PBYTE"},
	{0x5000000B, 0x5000000B, "GPIO: P0_11 PBYTE"},
	{0x5000000C, 0x5000000C, "GPIO: P0_12 PBYTE"},
	{0x5000000D, 0x5000000D, "GPIO: P0_13 PBYTE"},
	{0x5000000E, 0x5000000E, "GPIO: P0_14 PBYTE"},
	{0x5000000F, 0x5000000F, "GPIO: P0_15 PBYTE"},
	{0x50000010, 0x50000010, "GPIO: P0_16 PBYTE"},
	{0x50000011, 0x50000011, "GPIO: P0_17 PBYTE"},
	{0x50000012, 0x50000012, "GPIO: P0_18 PBYTE"},
	{0x50000013, 0x50000013, "GPIO: P0_19 PBYTE"},
	{0x50000014, 0x50000014, "GPIO: P0_20 PBYTE"},
	{0x50000015, 0x50000015, "GPIO: P0_21 PBYTE"},
	{0x50000016, 0x50000016, "GPIO: P0_22 PBYTE"},
	{0x50000017, 0x50000017, "GPIO: P0_23 PBYTE"},
	{0x50000020, 0x50000020, "GPIO: P1_0 PBYTE"},
	{0x50000021, 0x50000021, "GPIO: P1_1 PBYTE"},
	{0x50000022, 0x50000022, "GPIO: P1_2 PBYTE"},
	{0x50000023, 0x50000023, "GPIO: P1_3 PBYTE"},
	{0x50000024, 0x50000024, "GPIO: P1_4 PBYTE"},
	{0x50000025, 0x50000025, "GPIO: P1_5 PBYTE"},
	{0x50000026, 0x50000026, "GPIO: P1_6 PBYTE"},
	{0x50000027, 0x50000027, "GPIO: P1_7 PBYTE"},
	{0x50000028, 0x50000028, "GPIO: P1_8 PBYTE"},
	{0x50000029, 0x50000029, "GPIO: P1_9 PBYTE"},
	{0x5000002A, 0x5000002A, "GPIO: P1_10 PBYTE"},
	{0x5000002B, 0x5000002B, "GPIO: P1_11 PBYTE"},
	{0x5000002C, 0x5000002C, "GPIO: P1_12 PBYTE"},
	{0x5000002D, 0x5000002D, "GPIO: P1_13 PBYTE"},
	{0x5000002E, 0x5000002E, "GPIO: P1_14 PBYTE"},
	{0x5000002F, 0x5000002F, "GPIO: P1_15 PBYTE"},
	{0x50000030, 0x50000030, "GPIO: P1_16 PBYTE"},
	{0x50000031, 0x50000031, "GPIO: P1_17 PBYTE"},
	{0x50000032, 0x50000032, "GPIO: P1_18 PBYTE"},
	{0x50000033, 0x50000033, "GPIO: P1_19 PBYTE"},
	{0x50000034, 0x50000034, "GPIO: P1_20 PBYTE"},
	{0x50000035, 0x50000035, "GPIO: P1_21 PBYTE"},
	{0x50000036, 0x50000036, "GPIO: P1_22 PBYTE"},
	{0x50000037, 0x50000037, "GPIO: P1_23 PBYTE"},
	{0x50000038, 0x50000038, "GPIO: P1_24 PBYT4"},
	{0x50000039, 0x50000039, "GPIO: P1_25 PBYTE"},
	{0x5000003A, 0x5000003A, "GPIO: P1_26 PBYTE"},
	{0x5000003B, 0x5000003B, "GPIO: P1_27 PBYTE"},
	{0x5000003C, 0x5000003C, "GPIO: P1_28 PBYTE"},
	{0x5000003D, 0x5000003D, "GPIO: P1_29 PBYTE"},
	{0x5000003E, 0x5000003E, "GPIO: P1_30 PBYTE"},
	{0x5000003F, 0x5000003F, "GPIO: P1_31 PBYTE"},
	{0x50001000, 0x50001004-1, "GPIO: P0_0 PWORD"}, 
	{0x50001004, 0x50001008-1, "GPIO: P0_1 PWORD"}, 
	{0x50001008, 0x5000100C-1, "GPIO: P0_2 PWORD"}, 
	{0x5000100C, 0x50001010-1, "GPIO: P0_3 PWORD"}, 
	{0x50001010, 0x50001014-1, "GPIO: P0_4 PWORD"}, 
	{0x50001014, 0x50001018-1, "GPIO: P0_5 PWORD"}, 
	{0x50001018, 0x5000101C-1, "GPIO: P0_6 PWORD"}, 
	{0x5000101C, 0x50001020-1, "GPIO: P0_7 PWORD"}, 
	{0x50001020, 0x50001024-1, "GPIO: P0_8 PWORD"}, 
	{0x50001024, 0x50001028-1, "GPIO: P0_9 PWORD"}, 
	{0x50001028, 0x5000102C-1, "GPIO: P0_10 PWORD"}, 
	{0x5000102C, 0x50001030-1, "GPIO: P0_11 PWORD"}, 
	{0x50001030, 0x50001034-1, "GPIO: P0_12 PWORD"}, 
	{0x50001034, 0x50001038-1, "GPIO: P0_13 PWORD"}, 
	{0x50001038, 0x5000103C-1, "GPIO: P0_14 PWORD"}, 
	{0x5000103C, 0x50001040-1, "GPIO: P0_15 PWORD"}, 
	{0x50001040, 0x50001044-1, "GPIO: P0_16 PWORD"}, 
	{0x50001044, 0x50001048-1, "GPIO: P0_17 PWORD"}, 
	{0x50001048, 0x5000104C-1, "GPIO: P0_18 PWORD"}, 
	{0x5000104C, 0x50001050-1, "GPIO: P0_19 PWORD"}, 
	{0x50001050, 0x50001054-1, "GPIO: P0_20 PWORD"}, 
	{0x50001054, 0x50001058-1, "GPIO: P0_21 PWORD"}, 
	{0x50001058, 0x5000105C-1, "GPIO: P0_22 PWORD"}, 
	{0x5000105C, 0x50001060-1, "GPIO: P0_23 PWORD"}, 
	{0x50001080, 0x50001084-1, "GPIO: P1_0 PWORD"}, 
	{0x50001084, 0x50001088-1, "GPIO: P1_1 PWORD"}, 
	{0x50001088, 0x5000108C-1, "GPIO: P1_2 PWORD"}, 
	{0x5000108C, 0x50001090-1, "GPIO: P1_3 PWORD"}, 
	{0x50001090, 0x50001094-1, "GPIO: P1_4 PWORD"}, 
	{0x50001094, 0x50001098-1, "GPIO: P1_5 PWORD"}, 
	{0x50001098, 0x5000109C-1, "GPIO: P1_6 PWORD"}, 
	{0x5000109C, 0x500010A0-1, "GPIO: P1_7 PWORD"}, 
	{0x500010A0, 0x500010A4-1, "GPIO: P1_8 PWORD"}, 
	{0x500010A4, 0x500010A8-1, "GPIO: P1_9 PWORD"}, 
	{0x500010A8, 0x500010AC-1, "GPIO: P1_10 PWORD"}, 
	{0x500010AC, 0x500010B0-1, "GPIO: P1_11 PWORD"}, 
	{0x500010B0, 0x500010B4-1, "GPIO: P1_12 PWORD"}, 
	{0x500010B4, 0x500010B8-1, "GPIO: P1_13 PWORD"}, 
	{0x500010B8, 0x500010BC-1, "GPIO: P1_14 PWORD"}, 
	{0x500010BC, 0x500010C0-1, "GPIO: P1_15 PWORD"}, 
	{0x500010C0, 0x500010C4-1, "GPIO: P1_16 PWORD"}, 
	{0x500010C4, 0x500010C8-1, "GPIO: P1_17 PWORD"}, 
	{0x500010C8, 0x500010CC-1, "GPIO: P1_18 PWORD"}, 
	{0x500010CC, 0x500010D0-1, "GPIO: P1_19 PWORD"}, 
	{0x500010D0, 0x500010D4-1, "GPIO: P1_20 PWORD"}, 
	{0x500010D4, 0x500010D8-1, "GPIO: P1_21 PWORD"}, 
	{0x500010D8, 0x500010DC-1, "GPIO: P1_22 PWORD"}, 
	{0x500010DC, 0x500010E0-1, "GPIO: P1_23 PWORD"}, 
	{0x500010E0, 0x500010E4-1, "GPIO: P1_24 PWORD"}, 
	{0x500010E4, 0x500010E8-1, "GPIO: P1_25 PWORD"}, 
	{0x500010E8, 0x500010EC-1, "GPIO: P1_26 PWORD"}, 
	{0x500010EC, 0x500010F0-1, "GPIO: P1_27 PWORD"}, 
	{0x500010F0, 0x500010F4-1, "GPIO: P1_28 PWORD"}, 
	{0x500010F4, 0x500010F8-1, "GPIO: P1_29 PWORD"}, 
	{0x500010F8, 0x500010FC-1, "GPIO: P1_30 PWORD"}, 
	{0x500010FC, 0x50001100-1, "GPIO: P1_31 PWORD"}, 
	{0x50002000, 0x50002004-1, "GPIO: DIR0"}, 
	{0x50002004, 0x50002008-1, "GPIO: DIR1"}, 
	{0x50002080, 0x50002084-1, "GPIO: MASK0"}, 
	{0x50002084, 0x50002088-1, "GPIO: MASK1"}, 
	{0x50002100, 0x50002104-1, "GPIO: PIN0"}, 
	{0x50002104, 0x50002108-1, "GPIO: PIN1"}, 
	{0x50002180, 0x50002184-1, "GPIO: MPIN0"}, 
	{0x50002184, 0x50002188-1, "GPIO: MPIN1"}, 
	{0x50002200, 0x50002204-1, "GPIO: SET0"}, 
	{0x50002204, 0x50002208-1, "GPIO: SET1"}, 
	{0x50002280, 0x50002284-1, "GPIO: CLR0"}, 
	{0x50002284, 0x50002288-1, "GPIO: CLR1"}, 
	{0x50002300, 0x50002304-1, "GPIO: NOT0"}, 
	{0x50002304, 0x50002308-1, "GPIO: NOT1"}, 

	//{0xE0000000, 0xE0100000-1, "private peripheral bus"}, // --ram 3758096384 1048576
	{0xE000E010, 0xE000E014-1, "SysTick timer: SYST_CSR"},
	{0xE000E014, 0xE000E018-1, "SysTick timer: SYST_RVR"},
	{0xE000E018, 0xE000E01C-1, "SysTick timer: SYST_CVR"},
	{0xE000E01C, 0xE000E020-1, "SysTick timer: SYST_CALIB"},

	{0xE000E100, 0xE000E104-1, "NVIC: ISER0"},
	{0xE000E180, 0xE000E184-1, "NVIC: ICER0"},
	{0xE000E200, 0xE000E204-1, "NVIC: ISPR0"},
	{0xE000E280, 0xE000E284-1, "NVIC: ICPR0"},
	{0xE000E300, 0xE000E304-1, "NVIC: IABR0"},
	{0xE000E400, 0xE000E404-1, "NVIC: IPR0"},
	{0xE000E404, 0xE000E408-1, "NVIC: IPR1"},
	{0xE000E408, 0xE000E40C-1, "NVIC: IPR2"},
	{0xE000E40C, 0xE000E410-1, "NVIC: IPR3"},
	{0xE000E410, 0xE000E414-1, "NVIC: IPR4"},
	{0xE000E414, 0xE000E418-1, "NVIC: IPR5"},
	{0xE000E418, 0xE000E41C-1, "NVIC: IPR6"},
	{0xE000E41C, 0xE000E420-1, "NVIC: IPR7"},

	{0xE000ED00, 0xE000ED04-1, "SCB: CPUID"},
	{0xE000ED04, 0xE000ED08-1, "SCB: ICSR"},
	{0xE000ED0C, 0xE000ED10-1, "SCB: AIRCR"},
	{0xE000ED10, 0xE000ED14-1, "SCB: SCR"},
	{0xE000ED14, 0xE000ED18-1, "SCB: CCR"},
	{0xE000ED1C, 0xE000ED20-1, "SCB: SHPR2"},
	{0xE000ED20, 0xE000ED24-1, "SCB: SHPR3"},

	{0xE0100000, 0xFFFFFFFF-1, "reserved"}
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
		if (addr >= memInfo[cnt].start && addr <= memInfo[cnt].end)
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

static const uint32_t MAX_DESC_STR_LEN = 128 + 32; //!< Maximum length of description
	//!< field in exe log.

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
	logExeIncIndentCCode();

	fflush(exeLogCFile);
}

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
