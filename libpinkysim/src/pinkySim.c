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
#include <assert.h>
#include <common.h>
#include <MallocFailureInject.h>
#include <pinkySim.h>
#include <pinkySimLogExe.h>

/* Fields decoded from instructions. */
typedef struct Fields
{
    uint32_t imm;
    uint32_t d;
    uint32_t m;
    uint32_t n;
    uint32_t t;
    uint32_t registers;
} Fields;

/* Types of shift/rotate operations. */
typedef enum SRType
{
    SRType_LSL,
    SRType_LSR,
    SRType_ASR,
    // NOTE: ARMv6-M doesn't use the RRX type.
    //SRType_RRX,
    SRType_ROR
} SRType;

/* Decoded shift/rotate operation. */
typedef struct DecodedImmShift
{
    SRType   type;
    uint32_t n;
} DecodedImmShift;

/* Shift operation results. */
typedef struct ShiftResults
{
    uint32_t result;
    uint32_t carryOut;
} ShiftResults;

/* Results from addition/subtraction. */
typedef struct AddResults
{
    uint32_t result;
    int      carryOut;
    int      overflow;
} AddResults;

/* Function Prototypes */
static int executeInstruction16(PinkySimContext* pContext, uint16_t instr);
static int shiftAddSubtractMoveCompare(PinkySimContext* pContext, uint16_t instr);
static int lslImmediate(PinkySimContext* pContext, uint16_t instr);
static Fields decodeImm10to6_Rm5to3_Rd2to0(uint32_t instr);
static DecodedImmShift decodeImmshift(uint32_t typeBits, uint32_t imm5);
static ShiftResults shift_C(uint32_t value, SRType type, uint32_t amount, uint32_t carryIn);
static ShiftResults LSL_C(uint32_t x, uint32_t shift);
static ShiftResults LSR_C(uint32_t x, uint32_t shift);
static ShiftResults ASR_C(uint32_t x, uint32_t shift);
static ShiftResults ROR_C(uint32_t x, uint32_t shift);
static uint32_t LSR(uint32_t x, uint32_t shift);
static uint32_t LSL(uint32_t x, uint32_t shift);
static uint32_t getReg(const PinkySimContext* pContext, uint32_t reg);
static void setReg(PinkySimContext* pContext, uint32_t reg, uint32_t value);
static void updateRdAndNZC(PinkySimContext* pContext, Fields* pFields, const ShiftResults* pShiftResults);
static int lsrImmediate(PinkySimContext* pContext, uint16_t instr);
static int asrImmediate(PinkySimContext* pContext, uint16_t instr);
static int addRegisterT1(PinkySimContext* pContext, uint16_t instr);
static Fields decodeRm8to6_Rn5to3_Rd2to0(uint32_t instr);
static AddResults addWithCarry(uint32_t x, uint32_t y, uint32_t carryInAsBit);
static void updateRdAndNZCV(PinkySimContext* pContext, Fields* pFields, const AddResults* pAddResults);
static void updateNZCV(PinkySimContext* pContext, Fields* pFields, const AddResults* pAddResults);
static int subRegister(PinkySimContext* pContext, uint16_t instr);
static int addImmediateT1(PinkySimContext* pContext, uint16_t instr);
static Fields decodeImm8to6_Rn5to3_Rd2to0(uint32_t instr);
static int subImmediateT1(PinkySimContext* pContext, uint16_t instr);
static int movImmediate(PinkySimContext* pContext, uint16_t instr);
static void updateRdAndNZ(PinkySimContext* pContext, Fields* pFields, uint32_t results);
static void updateNZ(PinkySimContext* pContext, Fields* pFields, uint32_t results);
static Fields decodeRd10to8_Imm7to0(uint32_t instr);
static int cmpImmediate(PinkySimContext* pContext, uint16_t instr);
static Fields decodeRn10to8_Imm7to0(uint32_t instr);
static int addImmediateT2(PinkySimContext* pContext, uint16_t instr);
static Fields decodeRdn10to8_Imm7to0(uint32_t instr);
static int subImmediateT2(PinkySimContext* pContext, uint16_t instr);
static int dataProcessing(PinkySimContext* pContext, uint16_t instr);
static int andRegister(PinkySimContext* pContext, uint16_t instr);
static Fields decodeRm5to3_Rdn2to0(uint32_t instr);
static int eorRegister(PinkySimContext* pContext, uint16_t instr);
static int lslRegister(PinkySimContext* pContext, uint16_t instr);
static int lsrRegister(PinkySimContext* pContext, uint16_t instr);
static int asrRegister(PinkySimContext* pContext, uint16_t instr);
static int adcRegister(PinkySimContext* pContext, uint16_t instr);
static int sbcRegister(PinkySimContext* pContext, uint16_t instr);
static int rorRegister(PinkySimContext* pContext, uint16_t instr);
static int tstRegister(PinkySimContext* pContext, uint16_t instr);
static int rsbRegister(PinkySimContext* pContext, uint16_t instr);
static Fields decodeRn5to3_Rd2to0(uint32_t instr);
static int cmpRegisterT1(PinkySimContext* pContext, uint16_t instr);
static int cmnRegister(PinkySimContext* pContext, uint16_t instr);
static int orrRegister(PinkySimContext* pContext, uint16_t instr);
static int mulRegister(PinkySimContext* pContext, uint16_t instr);
static Fields decodeRn5to3_Rdm2to0(uint32_t instr);
static int bicRegister(PinkySimContext* pContext, uint16_t instr);
static int mvnRegister(PinkySimContext* pContext, uint16_t instr);
static int specialDataAndBranchExchange(PinkySimContext* pContext, uint16_t instr);
static int addRegisterT2(PinkySimContext* pContext, uint16_t instr);
static Fields decodeRdn7and2to0_Rm6to3(uint32_t instr);
static void aluWritePC(PinkySimContext* pContext, uint32_t address);
static void branchWritePC(PinkySimContext* pContext, uint32_t address);
static void branchTo(PinkySimContext* pContext, uint32_t address);
static int cmpRegisterT2(PinkySimContext* pContext, uint16_t instr);
static int movRegister(PinkySimContext* pContext, uint16_t instr);
static int bx(PinkySimContext* pContext, uint16_t instr);
static void bxWritePC(PinkySimContext* pContext, uint32_t address);
static int blx(PinkySimContext* pContext, uint16_t instr);
static void blxWritePC(PinkySimContext* pContext, uint32_t address);
static int ldrLiteral(PinkySimContext* pContext, uint16_t instr);
static Fields decodeRt10to8_Imm7to0Shift2(uint32_t instr);
static uint32_t align(uint32_t value, uint32_t alignment);
static uint32_t unalignedMemRead(PinkySimContext* pContext, uint32_t address, uint32_t size);
static uint32_t alignedMemRead(PinkySimContext* pContext, uint32_t address, uint32_t size);
static int isAligned(uint32_t address, uint32_t size);
static int loadStoreSingleDataItem(PinkySimContext* pContext, uint16_t instr);
static int strRegister(PinkySimContext* pContext, uint16_t instr);
static Fields decodeRm8to6_Rn5to3_Rt2to0(uint32_t instr);
static void unalignedMemWrite(PinkySimContext* pContext, uint32_t address, uint32_t size, uint32_t value);
static void alignedMemWrite(PinkySimContext* pContext, uint32_t address, uint32_t size, uint32_t value);
static int strhRegister(PinkySimContext* pContext, uint16_t instr);
static int strbRegister(PinkySimContext* pContext, uint16_t instr);
static int ldrsbRegister(PinkySimContext* pContext, uint16_t instr);
static uint32_t signExtend8(uint32_t valueToExtend);
static int ldrRegister(PinkySimContext* pContext, uint16_t instr);
static int ldrhRegister(PinkySimContext* pContext, uint16_t instr);
static uint32_t zeroExtend16(uint32_t value);
static int ldrbRegister(PinkySimContext* pContext, uint16_t instr);
static uint32_t zeroExtend8(uint32_t value);
static int ldrshRegister(PinkySimContext* pContext, uint16_t instr);
static uint32_t signExtend16(uint32_t valueToExtend);
static int strImmediateT1(PinkySimContext* pContext, uint16_t instr);
static Fields decodeImm10to6_Rn5to3_Rt2to0(uint32_t instr);
static int ldrImmediateT1(PinkySimContext* pContext, uint16_t instr);
static int strbImmediate(PinkySimContext* pContext, uint16_t instr);
static int ldrbImmediate(PinkySimContext* pContext, uint16_t instr);
static int strhImmediate(PinkySimContext* pContext, uint16_t instr);
static int ldrhImmediate(PinkySimContext* pContext, uint16_t instr);
static int strImmediateT2(PinkySimContext* pContext, uint16_t instr);
static int ldrImmediateT2(PinkySimContext* pContext, uint16_t instr);
static int adr(PinkySimContext* pContext, uint16_t instr);
static int addSPT1(PinkySimContext* pContext, uint16_t instr);
static int misc16BitInstructions(PinkySimContext* pContext, uint16_t instr);
static int addSPT2(PinkySimContext* pContext, uint16_t instr);
static Fields decodeRdisSP_Imm6to0Shift2(uint32_t instr);
static int subSP(PinkySimContext* pContext, uint16_t instr);
static int sxth(PinkySimContext* pContext, uint16_t instr);
static int sxtb(PinkySimContext* pContext, uint16_t instr);
static int uxth(PinkySimContext* pContext, uint16_t instr);
static int uxtb(PinkySimContext* pContext, uint16_t instr);
static int push(PinkySimContext* pContext, uint16_t instr);
static uint32_t bitCount(uint32_t value);
static int cps(PinkySimContext* pContext, uint16_t instr);
static int currentModeIsPrivileged(PinkySimContext* pContext);
static int rev(PinkySimContext* pContext, uint16_t instr);
static int rev16(PinkySimContext* pContext, uint16_t instr);
static int revsh(PinkySimContext* pContext, uint16_t instr);
static int pop(PinkySimContext* pContext, uint16_t instr);
static void loadWritePC(PinkySimContext* pContext, uint32_t address);
static int hints(PinkySimContext* pContext, uint16_t instr);
static int nop(PinkySimContext* pContext, uint16_t instr);
static int yield(PinkySimContext* pContext, uint16_t instr);
static int wfe(PinkySimContext* pContext, uint16_t instr);
static int wfi(PinkySimContext* pContext, uint16_t instr);
static int sev(PinkySimContext* pContext, uint16_t instr);
static int treatAsNop(PinkySimContext* pContext, uint16_t instr);
static int stm(PinkySimContext* pContext, uint16_t instr);
static Fields decodeRn10to8RegisterList7to0(uint32_t instr);
static int isNotLowestBitSet(uint32_t bits, uint32_t i);
static int ldm(PinkySimContext* pContext, uint16_t instr);
static int conditionalBranchAndSupervisor(PinkySimContext* pContext, uint16_t instr);
static int svc(PinkySimContext* pContext, uint16_t instr);
static int conditionalBranch(PinkySimContext* pContext, uint16_t instr);
static int conditionPassedForBranchInstr(PinkySimContext* pContext, uint16_t instr);
static int unconditionalBranch(PinkySimContext* pContext, uint16_t instr);
static int executeInstruction32(PinkySimContext* pContext, uint16_t instr1);
static int branchAndMiscellaneousControl(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2);
static int msr(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2);
static int miscellaneousControl(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2);
static int dsb(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2);
static int dmb(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2);
static int isb(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2);
static int mrs(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2);
static int bl(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2);


int pinkySimRun(PinkySimContext* pContext, int (*callback)(PinkySimContext*))
{
    int result;

    do
    {
        result = callback ? callback(pContext) : PINKYSIM_STEP_OK;
        if (result == PINKYSIM_STEP_OK)
            result = pinkySimStep(pContext);
    } while (result == PINKYSIM_STEP_OK);
    return result;
}


int pinkySimStep(PinkySimContext* pContext)
{
    int      result = PINKYSIM_STEP_UNDEFINED;

    if (!(pContext->xPSR & EPSR_T))
        return PINKYSIM_STEP_HARDFAULT;

    pContext->stepNum++;

    __try
    {
        uint16_t instr =  IMemory_Read16(pContext->pMemory, pContext->pc);

        if ((instr & 0xF800) == 0xE800 ||
            (instr & 0xF800) == 0xF000 ||
            (instr & 0xF800) == 0xF800)
            result = executeInstruction32(pContext, instr);
        else
            result = executeInstruction16(pContext, instr);
        pContext->pc = pContext->newPC;
    }
    __catch
    {
        switch (getExceptionCode())
        {
        case bkptException:
        case hardwareBreakpointException:
            return PINKYSIM_STEP_BKPT;
        case undefinedException:
            return PINKYSIM_STEP_UNDEFINED;
        case unpredictableException:
            return PINKYSIM_STEP_UNPREDICTABLE;
        default:
            return PINKYSIM_STEP_HARDFAULT;
        }
    }
    return result;
}

static int executeInstruction16(PinkySimContext* pContext, uint16_t instr)
{
    int result = PINKYSIM_STEP_UNDEFINED;

    pContext->newPC = pContext->pc + 2;
    if ((instr & 0xC000) == 0x0000)
        result = shiftAddSubtractMoveCompare(pContext, instr);
    else if ((instr & 0xFC00) == 0x4000)
        result = dataProcessing(pContext, instr);
    else if ((instr & 0xFC00) == 0x4400)
        result = specialDataAndBranchExchange(pContext, instr);
    else if ((instr & 0xF800) == 0x4800)
        result = ldrLiteral(pContext, instr);
    else if (((instr & 0xF000) == 0x5000) || ((instr & 0xE000) == 0x6000) || ((instr & 0xE000) == 0x8000))
        result = loadStoreSingleDataItem(pContext, instr);
    else if ((instr & 0xF800) == 0xA000)
        result = adr(pContext, instr);
    else if ((instr & 0xF800) == 0xA800)
        result = addSPT1(pContext, instr);
    else if ((instr & 0xF000) == 0xB000)
        result = misc16BitInstructions(pContext, instr);
    else if ((instr & 0xF800) == 0xC000)
        result = stm(pContext, instr);
    else if ((instr & 0xF800) == 0xC800)
        result = ldm(pContext, instr);
    else if ((instr & 0xF000) == 0xD000)
        result = conditionalBranchAndSupervisor(pContext, instr);
    else if ((instr & 0xF800) == 0xE000)
        result = unconditionalBranch(pContext, instr);
    return result;
}

static int shiftAddSubtractMoveCompare(PinkySimContext* pContext, uint16_t instr)
{
    int result = PINKYSIM_STEP_UNDEFINED;

    if ((instr & 0x3800) == 0x0000)
        result = lslImmediate(pContext, instr);
    else if ((instr & 0x3800) == 0x0800)
        result = lsrImmediate(pContext, instr);
    else if ((instr & 0x3800) == 0x1000)
        result = asrImmediate(pContext, instr);
    else if ((instr & 0x3E00) == 0x1800)
        result = addRegisterT1(pContext, instr);
    else if ((instr & 0x3E00) == 0x1A00)
        result = subRegister(pContext, instr);
    else if ((instr & 0x3E00) == 0x1C00)
        result = addImmediateT1(pContext, instr);
    else if ((instr & 0x3E00) == 0x1E00)
        result = subImmediateT1(pContext, instr);
    else if ((instr & 0x3800) == 0x2000)
        result = movImmediate(pContext, instr);
    else if ((instr & 0x3800) == 0x2800)
        result = cmpImmediate(pContext, instr);
    else if ((instr & 0x3800) == 0x3000)
        result = addImmediateT2(pContext, instr);
    else if ((instr & 0x3800) == 0x3800)
        result = subImmediateT2(pContext, instr);
    return result;
}

static int lslImmediate(PinkySimContext* pContext, uint16_t instr)
{
    Fields          fields = decodeImm10to6_Rm5to3_Rd2to0(instr);
    DecodedImmShift decodedShift = decodeImmshift(0x0, fields.imm);
    ShiftResults    shiftResults;
    uint32_t value_to_shift = getReg(pContext, fields.m);

    shiftResults = shift_C(value_to_shift, SRType_LSL, decodedShift.n, pContext->xPSR & APSR_C);
    updateRdAndNZC(pContext, &fields, &shiftResults);

    logExeInstr16(pContext, instr, "%s: Set Reg %d to Reg %d (0x%08x) shifted by %d to left (result = 0x%08x, carryOut = 0x%08x)", 
        __func__, fields.d, fields.m, value_to_shift, decodedShift.n, shiftResults.result, shiftResults.carryOut);

    logExeCCode("// 0x%08x = 0x%08x << %d (Carry Out = 0x%08x)\n",
        shiftResults.result, value_to_shift, decodedShift.n, shiftResults.carryOut);
    logExeCCode("reg%d = (uint32_t)reg%d << %d;\n\n", 
        fields.d, fields.m, decodedShift.n);

    return PINKYSIM_STEP_OK;
}

static Fields decodeImm10to6_Rm5to3_Rd2to0(uint32_t instr)
{
    Fields fields;

    fields.imm = (instr & (0x1F << 6)) >> 6;
    fields.d = instr & 0x7;
    fields.m = (instr & (0x7 << 3)) >> 3;
    return fields;
}

static DecodedImmShift decodeImmshift(uint32_t typeBits, uint32_t imm5)
{
    DecodedImmShift results;

    assert ((typeBits & 0x3) == typeBits);
    switch (typeBits)
    {
    case 0x0:
        results.type = SRType_LSL;
        results.n = imm5;
        break;
    case 0x1:
        results.type = SRType_LSR;
        results.n = (imm5 == 0) ? 32 : imm5;
        break;
    // NOTE: ARMv6-M only uses the first 3 decodings.
    //case 0x2:
    default:
        results.type = SRType_ASR;
        results.n = (imm5 == 0) ? 32 : imm5;
        break;
    }
    return results;
}

static ShiftResults shift_C(uint32_t value, SRType type, uint32_t amount, uint32_t carryIn)
{
    ShiftResults results = {value, carryIn};

    // NOTE: ARMv6-M doesn't use the RRX type.
    if (amount != 0)
    {
        switch (type)
        {
        case SRType_LSL:
            results = LSL_C(value, amount);
            break;
        case SRType_LSR:
            results = LSR_C(value, amount);
            break;
        case SRType_ASR:
            results = ASR_C(value, amount);
            break;
        case SRType_ROR:
            results = ROR_C(value, amount);
            break;
        }
    }
    return results;
}

static ShiftResults LSL_C(uint32_t x, uint32_t shift)
{
    ShiftResults results;

    results.carryOut = (shift > 32) ? 0 : x & (1 << (32 - shift));
    results.result = (shift > 31) ? 0 : x << shift;
    return results;
}

static ShiftResults LSR_C(uint32_t x, uint32_t shift)
{
    ShiftResults results;

    assert (shift > 0);

    results.carryOut = (shift > 32) ? 0 : (x & (1 << (shift - 1)));
    results.result = (shift > 31) ? 0 : x >> shift;
    return results;
}

static ShiftResults ASR_C(uint32_t x, uint32_t shift)
{
    ShiftResults results;

    assert (shift > 0);

    if (shift > 32)
    {
        if (x & 0x80000000)
            results.carryOut = 1;
        else
            results.carryOut = 0;
    }
    else
    {
        results.carryOut = (x & (1 << (shift - 1)));
    }
    if (shift > 31)
    {
        if (x & 0x80000000)
            results.result = 0xFFFFFFFF;
        else
            results.result = 0x00000000;
    }
    else
    {
        results.result = (uint32_t)((int32_t)x >> shift);
    }
    return results;
}

static ShiftResults ROR_C(uint32_t x, uint32_t shift)
{
    uint32_t     m;
    ShiftResults results;

    assert (shift != 0);

    m = shift & 31U;
    results.result = LSR(x, m) | LSL(x, 32-m);
    results.carryOut = results.result & (1 << 31);
    return results;
}

static uint32_t LSR(uint32_t x, uint32_t shift)
{
    ShiftResults results = {0, 0};

    assert (shift >= 0);

    if (shift == 0)
        results.result = x;
    else
        results = LSR_C(x, shift);
    return results.result;
}

static uint32_t LSL(uint32_t x, uint32_t shift)
{
    ShiftResults results = {x, 0};

    assert (shift >= 0);

    if (shift != 0)
        results = LSL_C(x, shift);
    return results.result;
}

static uint32_t getReg(const PinkySimContext* pContext, uint32_t reg)
{
    assert (reg <= PC);

    if (reg == PC)
        return pContext->pc + 4;
    else if (reg == LR)
        return pContext->lr;
    else if (reg == SP)
        // NOTE: Simulator only supports main handler mode.
        return pContext->spMain;
    else
        return pContext->R[reg];
}

static void setReg(PinkySimContext* pContext, uint32_t reg, uint32_t value)
{
    assert (reg < PC);

    if (reg == LR)
        pContext->lr = value;
    else if (reg == SP)
        // NOTE: Simulator only supports main handler mode.
        pContext->spMain = value;
    else
        pContext->R[reg] = value;
}

static void updateRdAndNZC(PinkySimContext* pContext, Fields* pFields, const ShiftResults* pShiftResults)
{
    setReg(pContext, pFields->d, pShiftResults->result);
    pContext->xPSR &= ~APSR_NZC;
    if (pShiftResults->result & (1 << 31))
        pContext->xPSR |= APSR_N;
    if (pShiftResults->result == 0)
        pContext->xPSR |= APSR_Z;
    if (pShiftResults->carryOut)
        pContext->xPSR |= APSR_C;
}

static int lsrImmediate(PinkySimContext* pContext, uint16_t instr)
{
    Fields          fields = decodeImm10to6_Rm5to3_Rd2to0(instr);
    DecodedImmShift decodedShift = decodeImmshift(0x1, fields.imm);
    ShiftResults    shiftResults;
    uint32_t value_to_shift = getReg(pContext, fields.m);

    shiftResults = shift_C(value_to_shift, SRType_LSR, decodedShift.n, pContext->xPSR & APSR_C);
    updateRdAndNZC(pContext, &fields, &shiftResults);

    logExeInstr16(pContext, instr, "%s: Set Reg %d to Reg %d (0x%08x) shifted by %d to right (result = 0x%08x, carryOut = 0x%08x)", 
        __func__, fields.d, fields.m, value_to_shift, decodedShift.n, shiftResults.result, shiftResults.carryOut);

    logExeCCode("// 0x%08x = 0x%08x >> %d (Carry Out = 0x%08x)\n",
        shiftResults.result, value_to_shift, decodedShift.n, shiftResults.carryOut);
    logExeCCode("reg%d = (uint32_t)reg%d >> %d;\n\n", 
        fields.d, fields.m, decodedShift.n);

    return PINKYSIM_STEP_OK;
}

static int asrImmediate(PinkySimContext* pContext, uint16_t instr)
{
    Fields          fields = decodeImm10to6_Rm5to3_Rd2to0(instr);
    DecodedImmShift decodedShift = decodeImmshift(0x2, fields.imm);
    ShiftResults    shiftResults;
    uint32_t value_to_shift = getReg(pContext, fields.m);

    shiftResults = shift_C(value_to_shift, SRType_ASR, decodedShift.n, pContext->xPSR & APSR_C);
    updateRdAndNZC(pContext, &fields, &shiftResults);

    logExeInstr16(pContext, instr, "%s: Set Reg %d to Reg %d (0x%08x) shifted by %d to right (result = 0x%08x, carryOut = 0x%08x)", 
        __func__, fields.d, fields.m, value_to_shift, decodedShift.n, shiftResults.result, shiftResults.carryOut);

    logExeCCode("// 0x%08x = 0x%08x >> %d (Carry Out = 0x%08x)\n",
        shiftResults.result, value_to_shift, decodedShift.n, shiftResults.carryOut);
    logExeCCode("reg%d = (uint32_t)reg%d >> %d;\n\n", 
        fields.d, fields.m, decodedShift.n);

    return PINKYSIM_STEP_OK;
}

static int addRegisterT1(PinkySimContext* pContext, uint16_t instr)
{
    Fields fields = decodeRm8to6_Rn5to3_Rd2to0(instr);
    AddResults      addResults;
    uint32_t add_op1 = getReg(pContext, fields.n);
    uint32_t add_op2 = getReg(pContext, fields.m);

    addResults = addWithCarry(add_op1, add_op2, 0);
    updateRdAndNZCV(pContext, &fields, &addResults);

    logExeInstr16(pContext, instr, "%s: Reg %d (0x%08x) = Reg %d (0x%08x) + Reg %d (0x%08x)", 
        __func__, fields.d, addResults.result, fields.n, add_op1, fields.m, add_op2);

    logExeCCode("// 0x%08x = 0x%08x + 0x%08x\n", 
        addResults.result, add_op1, add_op2);
    logExeCCode("reg%d = reg%d + reg%d;\n\n", fields.d, fields.n, fields.m);

    return PINKYSIM_STEP_OK;
}

static Fields decodeRm8to6_Rn5to3_Rd2to0(uint32_t instr)
{
    Fields fields;

    fields.d = instr & 0x7;
    fields.n = (instr & (0x7 << 3)) >> 3;
    fields.m = (instr & (0x7 << 6)) >> 6;
    return fields;
}

static AddResults addWithCarry(uint32_t x, uint32_t y, uint32_t carryInAsBit)
{
    int        carryIn = carryInAsBit ? 1 : 0;
    uint64_t   unsignedSum = (uint64_t)x + (uint64_t)y + (uint64_t)carryIn;
    int64_t    signedSum = (int64_t)(int32_t)x + (int64_t)(int32_t)y + (int64_t)carryIn;
    uint32_t   result = (uint32_t)unsignedSum;
    AddResults results;

    results.result = result;
    results.carryOut = ((uint64_t)result == unsignedSum) ? 0 : 1;
    results.overflow = ((int64_t)(int32_t)result == signedSum) ? 0 : 1;
    return results;
}

static void updateRdAndNZCV(PinkySimContext* pContext, Fields* pFields, const AddResults* pAddResults)
{
    setReg(pContext, pFields->d, pAddResults->result);
    updateNZCV(pContext, pFields, pAddResults);
}

static void updateNZCV(PinkySimContext* pContext, Fields* pFields, const AddResults* pAddResults)
{
    pContext->xPSR &= ~APSR_NZCV;
    if (pAddResults->result & (1 << 31))
        pContext->xPSR |= APSR_N;
    if (pAddResults->result == 0)
        pContext->xPSR |= APSR_Z;
    if (pAddResults->carryOut)
        pContext->xPSR |= APSR_C;
    if (pAddResults->overflow)
        pContext->xPSR |= APSR_V;
}

static int subRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields fields = decodeRm8to6_Rn5to3_Rd2to0(instr);
    AddResults      addResults;
    uint32_t sub_op1 = getReg(pContext, fields.n);
    uint32_t sub_op2 = getReg(pContext, fields.m);

    addResults = addWithCarry(sub_op1, ~sub_op2, 1);
    updateRdAndNZCV(pContext, &fields, &addResults);

    logExeInstr16(pContext, instr, "%s: Reg %d (0x%08x) = Reg %d (0x%08x) - Reg %d (0x%08x)", 
        __func__, fields.d, addResults.result, fields.n, sub_op1, fields.m, sub_op2);

    logExeCCode("// 0x%08x = 0x%08x - 0x%08x\n", 
        addResults.result, sub_op1, sub_op2);
    logExeCCode("reg%d = reg%d - reg%d;\n\n", fields.d, fields.n, fields.m);
  
    return PINKYSIM_STEP_OK;
}

static int addImmediateT1(PinkySimContext* pContext, uint16_t instr)
{
    Fields     fields = decodeImm8to6_Rn5to3_Rd2to0(instr);
    AddResults addResults;
    uint32_t add_op1 = getReg(pContext, fields.n);

    addResults = addWithCarry(add_op1, fields.imm, 0);
    updateRdAndNZCV(pContext, &fields, &addResults);

    logExeInstr16(pContext, instr, "%s: Reg %d (0x%08x) = Reg %d (0x%08x) + value 0x%08x", 
        __func__, fields.d, addResults.result, fields.n, add_op1, fields.imm);

    logExeCCode("// 0x%08x = 0x%08x + 0x%08x\n", 
        addResults.result, add_op1, fields.imm);
    logExeCCode("reg%d = reg%d + 0x%08x;\n\n", fields.d, fields.n, fields.imm);
  
    return PINKYSIM_STEP_OK;
}

static Fields decodeImm8to6_Rn5to3_Rd2to0(uint32_t instr)
{
    Fields fields;

    fields.d = instr & 0x7;
    fields.n = (instr & (0x7 << 3)) >> 3;
    fields.imm = (instr & (0x7 << 6)) >> 6;
    return fields;
}

static int subImmediateT1(PinkySimContext* pContext, uint16_t instr)
{
    Fields     fields = decodeImm8to6_Rn5to3_Rd2to0(instr);
    AddResults addResults;
    uint32_t sub_op1 = getReg(pContext, fields.n);

    addResults = addWithCarry(sub_op1, ~fields.imm, 1);
    updateRdAndNZCV(pContext, &fields, &addResults);

    logExeInstr16(pContext, instr, "%s: Reg %d (0x%08x) = Reg %d (0x%08x) - value 0x%08x",
         __func__, fields.d, addResults.result, fields.n, sub_op1, fields.imm);

    logExeCCode("// 0x%08x = 0x%08x - 0x%08x\n", 
        addResults.result, sub_op1, fields.imm);
    logExeCCode("reg%d = reg%d - 0x%08x;\n\n", fields.d, fields.n, fields.imm);

    return PINKYSIM_STEP_OK;
}

static int movImmediate(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRd10to8_Imm7to0(instr);
    uint32_t result = fields.imm;

    updateRdAndNZ(pContext, &fields, result);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with value 0x%08x", 
        __func__, fields.d, result);

    logExeCCode("reg%d = 0x%08x; // (fields.imm)\n\n", fields.d, result);

    return PINKYSIM_STEP_OK;
}

static void updateRdAndNZ(PinkySimContext* pContext, Fields* pFields, uint32_t results)
{
    setReg(pContext, pFields->d, results);
    updateNZ(pContext, pFields, results);
}

static void updateNZ(PinkySimContext* pContext, Fields* pFields, uint32_t results)
{
    pContext->xPSR &= ~APSR_NZ;
    if (results & (1 << 31))
        pContext->xPSR |= APSR_N;
    if (results == 0)
        pContext->xPSR |= APSR_Z;
}

static Fields decodeRd10to8_Imm7to0(uint32_t instr)
{
    Fields fields;

    fields.d = (instr & (0x7 << 8)) >> 8;
    fields.imm = instr & 0xFF;
    return fields;
}

static int cmpImmediate(PinkySimContext* pContext, uint16_t instr)
{
    Fields     fields = decodeRn10to8_Imm7to0(instr);
    AddResults addResults;

    addResults = addWithCarry(getReg(pContext, fields.n), ~fields.imm, 1);

    logExeInstr16(pContext, instr, "%s: Subtract Reg %d (0x%08x) minus 0x%08x", 
        __func__, fields.n, getReg(pContext, fields.n), fields.imm);

    logExeCCode("// Compute 0x%08x - 0x%08x for compare\n", 
        getReg(pContext, fields.n), fields.imm);
    logExeCCode("if (reg%d - 0x%08x) is ", fields.n, fields.imm);

    updateNZCV(pContext, &fields, &addResults);

    return PINKYSIM_STEP_OK;
}

static Fields decodeRn10to8_Imm7to0(uint32_t instr)
{
    Fields fields;

    fields.n = (instr & (0x7 << 8)) >> 8;
    fields.imm = instr & 0xFF;
    return fields;
}

static int addImmediateT2(PinkySimContext* pContext, uint16_t instr)
{
    Fields     fields = decodeRdn10to8_Imm7to0(instr);
    AddResults addResults;

    addResults = addWithCarry(getReg(pContext, fields.n), fields.imm, 0);

    logExeInstr16(pContext, instr, "%s: Reg %d (0x%08x) = Reg %d (0x%08x) + 0x%08x", 
        __func__, fields.d, addResults.result, fields.n, getReg(pContext, fields.n), fields.imm);

    logExeCCode("// 0x%08x = 0x%08x + 0x%08x\n", addResults.result, 
        getReg(pContext, fields.n), fields.imm);
    logExeCCode("reg%d = reg%d + 0x%08x\n\n", fields.d, fields.n, fields.imm);

    updateRdAndNZCV(pContext, &fields, &addResults);

    return PINKYSIM_STEP_OK;
}

static Fields decodeRdn10to8_Imm7to0(uint32_t instr)
{
    Fields fields;

    fields.d = (instr & (0x7 << 8)) >> 8;
    fields.n = fields.d;
    fields.imm = instr & 0xFF;
    return fields;
}

static int subImmediateT2(PinkySimContext* pContext, uint16_t instr)
{
    Fields     fields = decodeRdn10to8_Imm7to0(instr);
    AddResults addResults;

    addResults = addWithCarry(getReg(pContext, fields.n), ~fields.imm, 1);

    logExeInstr16(pContext, instr, "%s: Reg %d (0x%08x) = Reg %d (0x%08x) - 0x%08x",
         __func__, fields.d, addResults.result, fields.n, getReg(pContext, fields.n), fields.imm);

    logExeCCode("// 0x%08x = 0x%08x - 0x%08x\n", addResults.result, 
        getReg(pContext, fields.n), fields.imm);
    logExeCCode("reg%d = reg%d - 0x%08x\n\n", fields.d, fields.n, fields.imm);

    updateRdAndNZCV(pContext, &fields, &addResults);

    return PINKYSIM_STEP_OK;
}

static int dataProcessing(PinkySimContext* pContext, uint16_t instr)
{
    int result = PINKYSIM_STEP_UNDEFINED;

    switch ((instr & 0x03C0) >> 6)
    {
    case 0:
        result = andRegister(pContext, instr);
        break;
    case 1:
        result = eorRegister(pContext, instr);
        break;
    case 2:
        result = lslRegister(pContext, instr);
        break;
    case 3:
        result = lsrRegister(pContext, instr);
        break;
    case 4:
        result = asrRegister(pContext, instr);
        break;
    case 5:
        result = adcRegister(pContext, instr);
        break;
    case 6:
        result = sbcRegister(pContext, instr);
        break;
    case 7:
        result = rorRegister(pContext, instr);
        break;
    case 8:
        result = tstRegister(pContext, instr);
        break;
    case 9:
        result = rsbRegister(pContext, instr);
        break;
    case 10:
        result = cmpRegisterT1(pContext, instr);
        break;
    case 11:
        result = cmnRegister(pContext, instr);
        break;
    case 12:
        result = orrRegister(pContext, instr);
        break;
    case 13:
        result = mulRegister(pContext, instr);
        break;
    case 14:
        result = bicRegister(pContext, instr);
        break;
    case 15:
        result = mvnRegister(pContext, instr);
        break;
    }
    return result;
}

static int andRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t result;

    result = getReg(pContext, fields.n) & getReg(pContext, fields.m);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with value 0x%08x (Reg %d & Reg %d)", 
        __func__, fields.d, result, fields.n, fields.m);

    logExeCCode("// 0x%08x = 0x%08x & 0x%08x\n", result, 
        getReg(pContext, fields.n), getReg(pContext, fields.m));
    logExeCCode("reg%d = reg%d & reg%d;\n\n", fields.d, fields.n, fields.m);

    updateRdAndNZ(pContext, &fields, result);

    return PINKYSIM_STEP_OK;
}

static Fields decodeRm5to3_Rdn2to0(uint32_t instr)
{
    Fields fields;

    fields.d = instr & 0x7;
    fields.n = fields.d;
    fields.m = (instr & (0x7 << 3)) >> 3;
    return fields;
}

static int eorRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t result;

    result = getReg(pContext, fields.n) ^ getReg(pContext, fields.m);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with value 0x%08x (Reg %d ^ Reg %d)", 
        __func__, fields.d, result, fields.n, fields.m);

    logExeCCode("// 0x%08x = 0x%08x ^ 0x%08x\n", result, 
        getReg(pContext, fields.n), getReg(pContext, fields.m));
    logExeCCode("reg%d = reg%d ^ reg%d;\n\n", fields.d, fields.n, fields.m);

    updateRdAndNZ(pContext, &fields, result);

    return PINKYSIM_STEP_OK;
}

static int lslRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields       fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t     shiftN;
    ShiftResults shiftResults;
    uint32_t value_to_shift = getReg(pContext, fields.n);

    shiftN = getReg(pContext, fields.m) & 0xFF;
    shiftResults = shift_C(value_to_shift, SRType_LSL, shiftN, pContext->xPSR & APSR_C);
    updateRdAndNZC(pContext, &fields, &shiftResults);

    logExeInstr16(pContext, instr, "%s: Set Reg %d to Reg %d (0x%08x) shifted by %d (Value of Reg %d) to left (result = 0x%08x, carryOut = 0x%08x)", 
        __func__, fields.d, fields.n, value_to_shift, shiftN, fields.m, shiftResults.result, shiftResults.carryOut);

    logExeCCode("// 0x%08x = 0x%08x << 0x%08x (Carry Out = 0x%08x)\n", 
        shiftResults.result, value_to_shift, shiftN, shiftResults.carryOut);
    logExeCCode("reg%d = (uint32_t)reg%d << reg%d;\n\n", 
        fields.d, fields.n, fields.m);

    return PINKYSIM_STEP_OK;
}

static int lsrRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields       fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t     shiftN;
    ShiftResults shiftResults;
    uint32_t value_to_shift = getReg(pContext, fields.n);

    shiftN = getReg(pContext, fields.m) & 0xFF;
    shiftResults = shift_C(value_to_shift, SRType_LSR, shiftN, pContext->xPSR & APSR_C);
    updateRdAndNZC(pContext, &fields, &shiftResults);

    logExeInstr16(pContext, instr, "%s: Set Reg %d to Reg %d (0x%08x) shifted by %d (Value of Reg %d) to right (result = 0x%08x, carryOut = 0x%08x)", 
        __func__, fields.d, fields.n, value_to_shift, shiftN, fields.m, shiftResults.result, shiftResults.carryOut);

    logExeCCode("// 0x%08x = 0x%08x >> 0x%08x (Carry Out = 0x%08x)\n", 
        shiftResults.result, value_to_shift, shiftN, shiftResults.carryOut);
    logExeCCode("reg%d = (uint32_t)reg%d >> reg%d;\n\n", 
        fields.d, fields.n, fields.m);

    return PINKYSIM_STEP_OK;
}

static int asrRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields       fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t     shiftN;
    ShiftResults shiftResults;
    uint32_t value_to_shift = getReg(pContext, fields.n);

    shiftN = getReg(pContext, fields.m) & 0xFF;
    shiftResults = shift_C(value_to_shift, SRType_ASR, shiftN, pContext->xPSR & APSR_C);
    updateRdAndNZC(pContext, &fields, &shiftResults);

    logExeInstr16(pContext, instr, "%s: Set Reg %d to Reg %d (0x%08x) shifted by %d (Value of Reg %d) to right (result = 0x%08x, carryOut = 0x%08x)", 
        __func__, fields.d, fields.n, value_to_shift, shiftN, fields.m, shiftResults.result, shiftResults.carryOut);

    logExeCCode("// 0x%08x = 0x%08x >> 0x%08x (Carry Out = 0x%08x)\n", 
        shiftResults.result, value_to_shift, shiftN, shiftResults.carryOut);
    logExeCCode("reg%d = (int32_t)reg%d >> reg%d;\n\n", 
        fields.d, fields.n, fields.m);

    return PINKYSIM_STEP_OK;
}

static int adcRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields     fields = decodeRm5to3_Rdn2to0(instr);
    AddResults addResults;
    uint32_t add1;
    uint32_t add2;
    uint32_t carry;

    add1 = getReg(pContext, fields.n);
    add2 = getReg(pContext, fields.m);
    carry = pContext->xPSR & APSR_C;
    addResults = addWithCarry(add1, add2, pContext->xPSR & APSR_C);
    updateRdAndNZCV(pContext, &fields, &addResults);

    logExeInstr16(pContext, instr, "%s: Reg %d (0x%08x) = (Reg %d (0x%08x) + Reg %d (0x%08x) + carry of %d)", 
        __func__, fields.d, addResults.result, fields.n, add1, fields.m, add2, carry);

    logExeCCode("// 0x%08x = 0x%08x + 0x%08x + 0x%08x\n", 
        addResults.result, add1, add2, carry);
    logExeCCode("reg%d = reg%d + reg%d + carry;\n\n", 
        fields.d, fields.n, fields.m);

    return PINKYSIM_STEP_OK;
}

static int sbcRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields      fields = decodeRm5to3_Rdn2to0(instr);
    AddResults  addResults;
    uint32_t add1;
    uint32_t add2;
    uint32_t carry;

    add1 = getReg(pContext, fields.n);
    add2 = getReg(pContext, fields.m);
    carry = pContext->xPSR & APSR_C;

    addResults = addWithCarry(add1, ~add2, pContext->xPSR & APSR_C);
    updateRdAndNZCV(pContext, &fields, &addResults);

    logExeInstr16(pContext, instr, "%s: Set Reg %d (0x%08x) = (Reg %d (0x%08x) + ~Reg %d (~0x%08x) + carry of %d)", 
        __func__, fields.d, addResults.result, fields.n, add1, fields.m, add2, carry);

    logExeCCode("// 0x%08x = 0x%08x + ~0x%08x + 0x%08x\n", 
        addResults.result, add1, add2, carry);
    logExeCCode("reg%d = reg%d + ~reg%d + carry;\n\n", 
        fields.d, fields.n, fields.m);

    return PINKYSIM_STEP_OK;
}

static int rorRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields       fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t     shiftN;
    ShiftResults shiftResults;
    uint32_t value_to_shift = getReg(pContext, fields.n);

    shiftN = getReg(pContext, fields.m) & 0xFF;
    shiftResults = shift_C(value_to_shift, SRType_ROR, shiftN, pContext->xPSR & APSR_C);
    updateRdAndNZC(pContext, &fields, &shiftResults);

    logExeInstr16(pContext, instr, "%s: Set Reg %d to Reg %d (0x%08x) rotated by %d (Value of Reg %d) to right (result = 0x%08x, carryOut = 0x%08x)", 
        __func__, fields.d, fields.n, value_to_shift, shiftN, fields.m, shiftResults.result, shiftResults.carryOut);

    logExeCCode("// 0x%08x = 0x%08x rotated by %d (Carry Out =  0x%08x)\n", 
        shiftResults.result, value_to_shift, shiftN, shiftResults.carryOut);
    logExeCCode("reg%d = reg%d ror red%d;\n\n", fields.d, fields.n, fields.m);

    return PINKYSIM_STEP_OK;
}

static int tstRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t result;
    uint32_t data1;
    uint32_t data2;

    data1 = getReg(pContext, fields.n);
    data2 = getReg(pContext, fields.m);
    result =  data1 & data2;
    updateNZ(pContext, &fields, result);

    logExeInstr16(pContext, instr, "%s: Logical AND Reg %d (0x%08x) and %d (0x%08x)", 
        __func__, fields.n, data1, fields.m, data2);

    logExeCCode("// Compute 0x%08x & 0x%08x for compare\n", data1, data2);
    logExeCCode("if (reg%d & reg%d) is ", fields.n, fields.m);

    return PINKYSIM_STEP_OK;
}

static int rsbRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields      fields = decodeRn5to3_Rd2to0(instr);
    uint32_t    imm32 = 0;
    AddResults  addResults;
    uint32_t    data;

    data = getReg(pContext, fields.n);
    addResults = addWithCarry(~data, imm32, 1);
    updateRdAndNZCV(pContext, &fields, &addResults);

    logExeInstr16(pContext, instr, "%s: Reg %d (0x%08x) = 0x%08x - Reg %d (0x%08x)", 
        __func__, fields.d, addResults.result, imm32, fields.n, data);

    logExeCCode("// 0x%08x  = 0x%08x - 08x%08x\n", addResults.result, imm32, data);
    logExeCCode("reg%d = 0x%08x - reg%d;\n\n", fields.d, imm32, fields.n);

    return PINKYSIM_STEP_OK;
}

static Fields decodeRn5to3_Rd2to0(uint32_t instr)
{
    Fields fields;

    fields.d = instr & 0x7;
    fields.n = (instr & (0x7 << 3)) >> 3;
    return fields;
}

static int cmpRegisterT1(PinkySimContext* pContext, uint16_t instr)
{
    Fields     fields = decodeRm5to3_Rdn2to0(instr);
    AddResults addResults;

    addResults = addWithCarry(getReg(pContext, fields.n), ~getReg(pContext, fields.m), 1);

    logExeInstr16(pContext, instr, "%s: Subtract Reg %d (0x%08x) minus Reg %d (0x%08x)", 
        __func__, fields.n, getReg(pContext, fields.n), fields.m, getReg(pContext, fields.m));

    logExeCCode("// Compute 0x%08x - 0x%08x for compare\n", 
        getReg(pContext, fields.n), getReg(pContext, fields.m));
    logExeCCode("if (reg%d - reg%d) is ", fields.n, fields.m);

    updateNZCV(pContext, &fields, &addResults);

    return PINKYSIM_STEP_OK;
}

static int cmnRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields     fields = decodeRm5to3_Rdn2to0(instr);
    AddResults addResults;

    addResults = addWithCarry(getReg(pContext, fields.n), getReg(pContext, fields.m), 0);

    logExeInstr16(pContext, instr, "%s: Add Reg %d (0x%08x) plus Reg %d (0x%08x)", 
        __func__, fields.n, getReg(pContext, fields.n), fields.m, getReg(pContext, fields.m));

    logExeCCode("// Compute 0x%08x + 0x%08x for compare\n", 
        getReg(pContext, fields.n), getReg(pContext, fields.m));
    logExeCCode("if (reg%d + reg%d) is ", fields.n, fields.m);

    updateNZCV(pContext, &fields, &addResults);

    return PINKYSIM_STEP_OK;
}

static int orrRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields     fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t   result;

    result = getReg(pContext, fields.n) | getReg(pContext, fields.m);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with value 0x%08x (Reg %d | Reg %d)",
        __func__, fields.d, result, fields.n, fields.m);

    logExeCCode("// 0x%08x = 0x%08x | 0x%08x;\n", result, 
        getReg(pContext, fields.n), getReg(pContext, fields.m));
    logExeCCode("reg%d = reg%d | reg%d;\n\n", fields.d, fields.n, fields.m);

    updateRdAndNZ(pContext, &fields, result);

    return PINKYSIM_STEP_OK;
}

static int mulRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields    fields = decodeRn5to3_Rdm2to0(instr);
    uint32_t  operand1 = getReg(pContext, fields.n);
    uint32_t  operand2 = getReg(pContext, fields.m);
    uint32_t  result = operand1 * operand2;

    logExeInstr16(pContext, instr, "%s: Set Reg %d with value 0x%08x (Reg %d * Reg %d)",
        __func__, fields.d, result, fields.n, fields.m);

    logExeCCode("// 0x%08x = 0x%08x * 0x%08x;\n", result, 
        getReg(pContext, fields.n), getReg(pContext, fields.m));
    logExeCCode("reg%d = reg%d * reg%d;\n\n", fields.d, fields.n, fields.m);

    updateRdAndNZ(pContext, &fields, result);

    return PINKYSIM_STEP_OK;
}

static Fields decodeRn5to3_Rdm2to0(uint32_t instr)
{
    Fields fields;

    fields.d = instr & 0x7;
    fields.m = fields.d;
    fields.n = (instr & (0x7 << 3)) >> 3;
    return fields;
}

static int bicRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t result;

    result = getReg(pContext, fields.n) & ~getReg(pContext, fields.m);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with value 0x%08x (Reg %d & ~Reg %d)", 
        __func__, fields.d, result, fields.n, fields.m);

    logExeCCode("// 0x%08x = 0x%08x & ~0x%08x;\n", result, 
        getReg(pContext, fields.n), getReg(pContext, fields.m));
    logExeCCode("reg%d = reg%d * ~reg%d;\n\n", fields.d, fields.n, fields.m);

    updateRdAndNZ(pContext, &fields, result);

    return PINKYSIM_STEP_OK;
}

static int mvnRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t result;

    result = ~getReg(pContext, fields.m);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with value 0x%08x (~Reg %d)", 
        __func__, fields.d, result, fields.m);

    logExeCCode("// 0x%08x = ~0x%08x;\n", result, getReg(pContext, fields.m));
    logExeCCode("reg%d = ~reg%d;\n\n", fields.d, fields.m);

    updateRdAndNZ(pContext, &fields, result);

    return PINKYSIM_STEP_OK;
}

static int specialDataAndBranchExchange(PinkySimContext* pContext, uint16_t instr)
{
    int result = PINKYSIM_STEP_UNDEFINED;

    if ((instr & 0x0300) == 0x0000)
        result = addRegisterT2(pContext, instr);
    else if ((instr & 0x03C0) == 0x0100)
    {
        logExeInstr16(pContext, instr, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException)
    }
    else if (((instr & 0x03C0) == 0x0140) || ((instr & 0x0380) == 0x0180))
        result = cmpRegisterT2(pContext, instr);
    else if ((instr & 0x0300) == 0x0200)
        result = movRegister(pContext, instr);
    else if ((instr & 0x0380) == 0x0300)
        result = bx(pContext, instr);
    else if ((instr & 0x0380) == 0x0380)
        result = blx(pContext, instr);
    return result;
}

static int addRegisterT2(PinkySimContext* pContext, uint16_t instr)
{
    Fields fields = decodeRdn7and2to0_Rm6to3(instr);
    AddResults      addResults;

    if (fields.d == 15 && fields.m == 15)
    {
        logExeInstr16(pContext, instr, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }

    addResults = addWithCarry(getReg(pContext, fields.n), getReg(pContext, fields.m), 0);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with value 0x%08x (Reg %d (%08x) + Reg %d (%08x))", 
        __func__, fields.d, addResults.result, fields.n, getReg(pContext, fields.n), fields.m, getReg(pContext, fields.m));

    logExeCCode("// 0x%08x = 0x%08x + 0x%08x\n", addResults.result,
        getReg(pContext, fields.n), getReg(pContext, fields.m));
    logExeCCode("reg%d = reg%d + reg%d\n\n", fields.d, fields.n, fields.m);

    if (fields.d == 15)
        aluWritePC(pContext, addResults.result);
    else
        setReg(pContext, fields.d, addResults.result);

    return PINKYSIM_STEP_OK;
}

static Fields decodeRdn7and2to0_Rm6to3(uint32_t instr)
{
    Fields fields;

    fields.d = ((instr & (1 << 7)) >> 4) | (instr & 0x7);
    fields.n = fields.d;
    fields.m = (instr & (0xF << 3)) >> 3;
    return fields;
}

static void aluWritePC(PinkySimContext* pContext, uint32_t address)
{
    branchWritePC(pContext, address);
}

static void branchWritePC(PinkySimContext* pContext, uint32_t address)
{
    branchTo(pContext, address & 0xFFFFFFFE);
}

static void branchTo(PinkySimContext* pContext, uint32_t address)
{
    pContext->newPC = address;
}

static int cmpRegisterT2(PinkySimContext* pContext, uint16_t instr)
{
    Fields fields = decodeRdn7and2to0_Rm6to3(instr);
    AddResults      addResults;

    if (fields.n == 15 || fields.m == 15)
    {
        logExeInstr16(pContext, instr, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }

    addResults = addWithCarry(getReg(pContext, fields.n), ~getReg(pContext, fields.m), 1);

    logExeInstr16(pContext, instr, "%s: Subtract Reg %d (0x%08x) minus Reg %d (0x%08x)", 
        __func__, fields.n, getReg(pContext, fields.n), fields.m, getReg(pContext, fields.m));

    logExeCCode("// Compute 0x%08x - 0x%08x for compare\n", 
        getReg(pContext, fields.n), getReg(pContext, fields.m));
    logExeCCode("if (reg%d - reg%d) is ", fields.n, fields.m);

    updateNZCV(pContext, &fields, &addResults);

    return PINKYSIM_STEP_OK;
}

static int movRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRdn7and2to0_Rm6to3(instr);
    uint32_t        result;

    result = getReg(pContext, fields.m);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with value from Reg %d (0x%08x)", 
        __func__, fields.d, fields.m, result);

    logExeCCode("reg%d = reg%d; // = 0x%08x\n\n", fields.d, fields.m, result);

    if (fields.d == 15)
        aluWritePC(pContext, result);
    else
        setReg(pContext, fields.d, result);

    return PINKYSIM_STEP_OK;
}

static int bx(PinkySimContext* pContext, uint16_t instr)
{
    Fields fields = decodeRdn7and2to0_Rm6to3(instr);
    uint32_t branchAddr;

    if ((instr & 0x7) != 0x0)
    {
        logExeInstr16(pContext, instr, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }
    if (fields.m == 15)
    {
        logExeInstr16(pContext, instr, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }

    branchAddr = getReg(pContext, fields.m);

    logExeInstr16(pContext, instr, "%s: Branch to Reg %d (0x%08x)", 
        __func__, fields.m, branchAddr);

    logExeCCode("// At 0x%08x branching to 0x%08x (reg%d)\n\n", 
        pContext->pc, branchAddr, fields.m);

    bxWritePC(pContext, branchAddr);

    return PINKYSIM_STEP_OK;
}

static void bxWritePC(PinkySimContext* pContext, uint32_t address)
{
    // NOTE: Don't support exception handlers in this simulator so no need to check for return from exception handler.
    //if (pContext->currentMode == modeHandler && (address & 0xF000) == 0xF000)
    //    ExceptionReturn(pContext, address & ((1 << 27) - 1));
    // else
    pContext->xPSR &= ~EPSR_T;
    if (address & 1)
        pContext->xPSR |= EPSR_T;
    branchTo(pContext, address & 0xFFFFFFFE);
}

static int blx(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRdn7and2to0_Rm6to3(instr);
    uint32_t target;
    uint32_t nextInstrAddr;

    if ((instr & 0x7) != 0x0)
    {
        logExeInstr16(pContext, instr, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }
    if (fields.m == 15)
    {
        logExeInstr16(pContext, instr, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }

    target = getReg(pContext, fields.m);
    nextInstrAddr = getReg(pContext, PC) - 2;

    logExeInstr16(pContext, instr, "%s: Set PC to Reg %d (0x%08x). Set LR to 0x%08x", 
        __func__, fields.m, target, nextInstrAddr);

    logExeCCode("// At 0x%08x branching to 0x%08x (reg%d). LR set to 0x%08x\n\n", 
        pContext->pc, target, fields.m, nextInstrAddr);

    setReg(pContext, LR, nextInstrAddr | 1);
    blxWritePC(pContext, target);

    return PINKYSIM_STEP_OK;
}

static void blxWritePC(PinkySimContext* pContext, uint32_t address)
{
    pContext->xPSR &= ~EPSR_T;
    if (address & 1)
        pContext->xPSR |= EPSR_T;
    branchTo(pContext, address & 0xFFFFFFFE);
}

static int ldrLiteral(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRt10to8_Imm7to0Shift2(instr);
    uint32_t base;
    uint32_t address;
    uint32_t value;

    base = align(getReg(pContext, PC), 4);
    address = base + fields.imm;
    value = unalignedMemRead(pContext, address, 4);
    setReg(pContext, fields.t, value);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with value 0x%08x (MemRead address 0x%08x - %s)", 
        __func__, fields.t, value, address, getMemInfo(address));

    logExeCCode("// MemRead %s (address encoded in instruction)\n", 
        getMemInfo(address));
    logExeCCode("reg%d = *(uint32_t*)0x%08x; // = 0x%08x\n\n", fields.t, 
        address, value);

    return PINKYSIM_STEP_OK;
}

static Fields decodeRt10to8_Imm7to0Shift2(uint32_t instr)
{
    Fields fields;

    fields.t = (instr & (0x7 << 8)) >> 8;
    fields.imm = (instr & 0xFF) << 2;
    return fields;
}

static uint32_t align(uint32_t value, uint32_t alignment)
{
    assert (alignment == 2 || alignment == 4);
    return value & ~(alignment - 1);
}

static uint32_t unalignedMemRead(PinkySimContext* pContext, uint32_t address, uint32_t size)
{
    // NOTE: All memory accesses must be aligned on ARMv6-M.
    return alignedMemRead(pContext, address, size);
}

static uint32_t alignedMemRead(PinkySimContext* pContext, uint32_t address, uint32_t size)
{
    uint32_t result = 0xFFFFFFFF;

    assert (size == 4 || size == 2 || size == 1);

    if (!isAligned(address, size))
    {
        __throw(alignmentException);
    }

    switch (size)
    {
    case 1:
        result = IMemory_Read8(pContext->pMemory, address);
        break;
    case 2:
        result = IMemory_Read16(pContext->pMemory, address);
        break;
    case 4:
        result = IMemory_Read32(pContext->pMemory, address);
        break;
    }

    return result;
}

static int isAligned(uint32_t address, uint32_t size)
{
    assert (size == 4 || size == 2 || size == 1);

    return address == (address & ~(size - 1));
}

static int loadStoreSingleDataItem(PinkySimContext* pContext, uint16_t instr)
{
    int result = PINKYSIM_STEP_UNDEFINED;

    if ((instr & 0xFE00) == 0x5000)
        result = strRegister(pContext, instr);
    else if ((instr & 0xFE00) == 0x5200)
        result = strhRegister(pContext, instr);
    else if ((instr & 0xFE00) == 0x5400)
        result = strbRegister(pContext, instr);
    else if ((instr & 0xFE00) == 0x5600)
        result = ldrsbRegister(pContext, instr);
    else if ((instr & 0xFE00) == 0x5800)
        result = ldrRegister(pContext, instr);
    else if ((instr & 0xFE00) == 0x5A00)
        result = ldrhRegister(pContext, instr);
    else if ((instr & 0xFE00) == 0x5C00)
        result = ldrbRegister(pContext, instr);
    else if ((instr & 0xFE00) == 0x5E00)
        result = ldrshRegister(pContext, instr);
    else if ((instr & 0xF800) == 0x6000)
        result = strImmediateT1(pContext, instr);
    else if ((instr & 0xF800) == 0x6800)
        result = ldrImmediateT1(pContext, instr);
    else if ((instr & 0xF800) == 0x7000)
        result = strbImmediate(pContext, instr);
    else if ((instr & 0xF800) == 0x7800)
        result = ldrbImmediate(pContext, instr);
    else if ((instr & 0xF800) == 0x8000)
        result = strhImmediate(pContext, instr);
    else if ((instr & 0xF800) == 0x8800)
        result = ldrhImmediate(pContext, instr);
    else if ((instr & 0xF800) == 0x9000)
        result = strImmediateT2(pContext, instr);
    else if ((instr & 0xF800) == 0x9800)
        result = ldrImmediateT2(pContext, instr);
    return result;
}

static int strRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm8to6_Rn5to3_Rt2to0(instr);
    uint32_t address;

    address = getReg(pContext, fields.n) + getReg(pContext, fields.m);
    uint32_t orig_val = unalignedMemRead(pContext, address, 4);
    unalignedMemWrite(pContext, address, 4, getReg(pContext, fields.t));

    uint32_t modified_bits = orig_val ^ getReg(pContext, fields.t);

    logExeInstr16(pContext, instr, "%s: MemWrite value of Reg %d (0x%08x) to address 0x%08x - %s (Reg %d (0x%08x) + Reg %d (0x%08x)). Modified Bits = 0x%08x",
        __func__, fields.t, getReg(pContext, fields.t), address, getMemInfo(address), 
        fields.n, getReg(pContext, fields.n), fields.m, getReg(pContext, fields.m),
        modified_bits);

    logExeCCode("// MemWrite %s (address was computed as reg%d + reg%d)\n", 
        getMemInfo(address), fields.n, fields.m);
    logExeCCode("*(uint32_t*)0x%08x = reg%d; // = 0x%08x (modified bits = 0x%08x)\n\n", 
        address, fields.t, getReg(pContext, fields.t), modified_bits);

    return PINKYSIM_STEP_OK;
}

static Fields decodeRm8to6_Rn5to3_Rt2to0(uint32_t instr)
{
    Fields fields;

    fields.t = instr & 0x7;
    fields.n = (instr & (0x7 << 3)) >> 3;
    fields.m = (instr & (0x7 << 6)) >> 6;
    return fields;
}

static void unalignedMemWrite(PinkySimContext* pContext, uint32_t address, uint32_t size, uint32_t value)
{
    // NOTE: All memory accesses must be aligned on ARMv6-M.
    alignedMemWrite(pContext, address, size, value);
}

static void alignedMemWrite(PinkySimContext* pContext, uint32_t address, uint32_t size, uint32_t value)
{
    assert (size == 4 || size == 2 || size == 1);

    if (!isAligned(address, size))
    {
        __throw(alignmentException);
    }

    switch (size)
    {
    case 4:
        IMemory_Write32(pContext->pMemory, address, value);
        break;
    case 2:
        IMemory_Write16(pContext->pMemory, address, value);
        break;
    case 1:
        IMemory_Write8(pContext->pMemory, address, value);
        break;
    }
}

static int strhRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm8to6_Rn5to3_Rt2to0(instr);
    uint32_t address;

    address = getReg(pContext, fields.n) + getReg(pContext, fields.m);
    uint32_t orig_val = unalignedMemRead(pContext, address, 2);
    unalignedMemWrite(pContext, address, 2, getReg(pContext, fields.t));

    uint32_t modified_bits = 0xFFFF&(orig_val ^ getReg(pContext, fields.t));

    logExeInstr16(pContext, instr, "%s: MemWrite 0x%04x (Reg %d) to 0x%08x - %s (Reg %d + Reg %d). Modified Bits = 0x%04x", 
        __func__, 0xFFFF&getReg(pContext, fields.t), fields.t, address, 
        getMemInfo(address), fields.n, fields.m, modified_bits);

    logExeCCode("// MemWrite %s (address was computed as reg%d + reg%d)\n", 
        getMemInfo(address), fields.n, fields.m);
    logExeCCode("*(uint16_t*)0x%08x = reg%d; // = 0x%04x (modified bits = 0x%04x)\n\n", 
        address, fields.t, 0xFFFF&getReg(pContext, fields.t), modified_bits);

    return PINKYSIM_STEP_OK;
}

static int strbRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm8to6_Rn5to3_Rt2to0(instr);
    uint32_t address;

    address = getReg(pContext, fields.n) + getReg(pContext, fields.m);
    uint32_t orig_val = unalignedMemRead(pContext, address, 1);
    unalignedMemWrite(pContext, address, 1, getReg(pContext, fields.t));

    uint32_t modified_bits = 0xFF&(orig_val ^ getReg(pContext, fields.t));

    logExeInstr16(pContext, instr, "%s: MemWrite 0x%02x (Reg %d) to 0x%08x - %s (Reg %d + Reg %d). Modified Bits = 0x%02x", 
        __func__, 0xFF&getReg(pContext, fields.t), fields.t, address, 
        getMemInfo(address), fields.n, fields.m, modified_bits);

    logExeCCode("// MemWrite %s (address was computed as reg%d + reg%d)\n", 
        getMemInfo(address), fields.n, fields.m);
    logExeCCode("*(uint8_t*)0x%08x = reg%d; // = 0x%02x (modified bits = 0x%02x)\n\n", 
        address, fields.t, 0xFF&getReg(pContext, fields.t), modified_bits);

    return PINKYSIM_STEP_OK;
}

static int ldrsbRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm8to6_Rn5to3_Rt2to0(instr);
    uint32_t address;
    uint32_t memRdVal = 0;

    address = getReg(pContext, fields.n) + getReg(pContext, fields.m);
    memRdVal = signExtend8(unalignedMemRead(pContext, address, 1)); 
    setReg(pContext, fields.t, memRdVal);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with 0x%08x (MemRead from 0x%08x - %s)", 
        __func__, fields.t, memRdVal, address, getMemInfo(address));

    logExeCCode("// MemRead %s (address was computed as reg%d + reg%d)\n", 
        getMemInfo(address), fields.n, fields.m);
    logExeCCode("reg%d = (int32_t)(*(int8_t*)0x%08x); // = 0x%08x\n\n", fields.t, 
        address, memRdVal);

    return PINKYSIM_STEP_OK;
}

static uint32_t signExtend8(uint32_t valueToExtend)
{
    return (uint32_t)(((int32_t)valueToExtend << 24) >> 24);
}

static int ldrRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm8to6_Rn5to3_Rt2to0(instr);
    uint32_t address;
    uint32_t memRdVal = 0;

    address = getReg(pContext, fields.n) + getReg(pContext, fields.m);
    memRdVal  = unalignedMemRead(pContext, address, 4);
    setReg(pContext, fields.t, memRdVal);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with 0x%08x (MemRead from 0x%08x - %s)", 
        __func__, fields.t, memRdVal, address, getMemInfo(address));

    logExeCCode("// MemRead %s (address was computed as reg%d + reg%d)\n", 
        getMemInfo(address), fields.n, fields.m);
    logExeCCode("reg%d = *(uint32_t*)0x%08x; // = 0x%08x\n\n", fields.t, 
        address, memRdVal);

    return PINKYSIM_STEP_OK;
}

static int ldrhRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm8to6_Rn5to3_Rt2to0(instr);
    uint32_t address;
    uint32_t data;

    address = getReg(pContext, fields.n) + getReg(pContext, fields.m);
    data = unalignedMemRead(pContext, address, 2);
    setReg(pContext, fields.t, zeroExtend16(data));

    logExeInstr16(pContext, instr, "%s: Set Reg %d with 0x%08x (MemRead from 0x%08x - %s)", 
        __func__, fields.t, data, address, getMemInfo(address));

    logExeCCode("// MemRead %s (address was computed as reg%d + reg%d)\n", 
        getMemInfo(address), fields.n, fields.m);
    logExeCCode("reg%d = *(uint16_t*)0x%08x; // = 0x%08x\n\n", fields.t, 
        address, data);

    return PINKYSIM_STEP_OK;
}

static uint32_t zeroExtend16(uint32_t value)
{
    return value & 0xFFFF;
}

static int ldrbRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm8to6_Rn5to3_Rt2to0(instr);
    uint32_t address;
    uint32_t data;

    address = getReg(pContext, fields.n) + getReg(pContext, fields.m);
    data = zeroExtend8(unalignedMemRead(pContext, address, 1));
    setReg(pContext, fields.t, data);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with 0x%08x (MemRead from 0x%08x - %s)", 
        __func__, fields.t, data, address, getMemInfo(address));

    logExeCCode("// MemRead %s (address was computed as reg%d + reg%d)\n", 
        getMemInfo(address), fields.n, fields.m);
    logExeCCode("reg%d = *(uint8_t*)0x%08x; // = 0x%08x\n\n", fields.t, 
        address, data);

    return PINKYSIM_STEP_OK;
}

static uint32_t zeroExtend8(uint32_t value)
{
    return value & 0xFF;
}

static int ldrshRegister(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm8to6_Rn5to3_Rt2to0(instr);
    uint32_t address;
    uint32_t data;

    address = getReg(pContext, fields.n) + getReg(pContext, fields.m);
    data = unalignedMemRead(pContext, address, 2);
    setReg(pContext, fields.t, signExtend16(data));

    logExeInstr16(pContext, instr, "%s: Set Reg %d with 0x%08x (MemRead from 0x%08x - %s)", 
        __func__, fields.t, signExtend16(data), address, getMemInfo(address));

    logExeCCode("// MemRead %s (address was computed as reg%d + reg%d)\n", 
        getMemInfo(address), fields.n, fields.m);
    logExeCCode("reg%d = *(uint16_t*)0x%08x; // = 0x%08x\n\n", fields.t, 
        address, signExtend16(data));

    return PINKYSIM_STEP_OK;
}

static uint32_t signExtend16(uint32_t valueToExtend)
{
    return (uint32_t)(((int32_t)valueToExtend << 16) >> 16);
}

static int strImmediateT1(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeImm10to6_Rn5to3_Rt2to0(instr);
    uint32_t address;

    address = getReg(pContext, fields.n) + (fields.imm << 2);
    uint32_t orig_val = unalignedMemRead(pContext, address, 4);
    unalignedMemWrite(pContext, address, 4, getReg(pContext, fields.t));

    uint32_t modified_bits = orig_val ^ getReg(pContext, fields.t);

    logExeInstr16(pContext, instr, "%s: MemWrite value of Reg %d (0x%08x) to address 0x%08x - %s (Reg %d (0x%08x) + 0x%08x). Modified Bits = 0x%08x", 
        __func__, fields.t, getReg(pContext, fields.t), address, getMemInfo(address), 
        fields.n, getReg(pContext, fields.n), (fields.imm << 2), modified_bits);

    logExeCCode("// MemWrite %s (address was computed as reg%d + 0x%08x)\n", 
        getMemInfo(address), fields.n, fields.imm << 2);
    logExeCCode("*(uint32_t*)0x%08x = reg%d; // = 0x%08x (modified bits = 0x%08x)\n\n", address, 
        fields.t, getReg(pContext, fields.t), modified_bits);

    return PINKYSIM_STEP_OK;
}

static Fields decodeImm10to6_Rn5to3_Rt2to0(uint32_t instr)
{
    Fields fields;

    fields.t = instr & 0x7;
    fields.n = (instr & (0x7 << 3)) >> 3;
    fields.imm = (instr & (0x1F << 6)) >> 6;
    return fields;
}

static int ldrImmediateT1(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeImm10to6_Rn5to3_Rt2to0(instr);
    uint32_t address;
    uint32_t value;

    address = getReg(pContext, fields.n) + (fields.imm << 2);
    value = unalignedMemRead(pContext, address, 4);
    setReg(pContext, fields.t, value);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with value 0x%08x (MemRead address 0x%08x - %s)", 
        __func__, fields.t, value, address, getMemInfo(address));

    logExeCCode("// MemRead %s (address was computed as reg%d + 0x%08x)\n", 
        getMemInfo(address), fields.n, fields.imm << 2);
    logExeCCode("reg%d = *(uint32_t*)0x%08x; // = 0x%08x\n\n", fields.t, 
        address, value);

    return PINKYSIM_STEP_OK;
}

static int strbImmediate(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeImm10to6_Rn5to3_Rt2to0(instr);
    uint32_t address;

    address = getReg(pContext, fields.n) + fields.imm;
    uint32_t orig_val = unalignedMemRead(pContext, address, 1);
    unalignedMemWrite(pContext, address, 1, getReg(pContext, fields.t));

    uint32_t modified_bits = 0xFF&(orig_val ^ getReg(pContext, fields.t));

    logExeInstr16(pContext, instr, "%s: MemWrite Reg %d (0x%02x) to 0x%08x - %s. Modified Bits = 0x%02x",
        __func__, fields.t, 0xFF&getReg(pContext, fields.t), address, 
        getMemInfo(address), modified_bits);

    logExeCCode("// MemWrite %s (address was computed as reg%d + 0x%08x)\n", 
        getMemInfo(address), fields.n, fields.imm);
    logExeCCode("*(uint8_t*)0x%08x = reg%d; // = 0x%02x (modified bits = 0x%02x)\n\n", 
        address, fields.t, 0xFF&getReg(pContext, fields.t), modified_bits);

    return PINKYSIM_STEP_OK;
}

static int ldrbImmediate(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeImm10to6_Rn5to3_Rt2to0(instr);
    uint32_t address;
    uint32_t data;

    address = getReg(pContext, fields.n) + fields.imm;
    data = unalignedMemRead(pContext, address, 1);
    setReg(pContext, fields.t, data);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with 0x%08x (MemRead from 0x%08x - %s)", 
        __func__, fields.t, data, address, getMemInfo(address));

    logExeCCode("// MemRead %s (address was computed as reg%d + 0x%08x)\n", 
        getMemInfo(address), fields.n, fields.imm);
    logExeCCode("reg%d = *(uint8_t*)0x%08x; // = 0x%08x\n\n", fields.t, 
        address, data);

    return PINKYSIM_STEP_OK;
}

static int strhImmediate(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeImm10to6_Rn5to3_Rt2to0(instr);
    uint32_t address;
    uint32_t data;

    address = getReg(pContext, fields.n) + (fields.imm << 1);
    data = getReg(pContext, fields.t);
    uint32_t orig_val = unalignedMemRead(pContext, address, 2);
    unalignedMemWrite(pContext, address, 2, data);

    uint32_t modified_bits = 0xFFFF&(orig_val ^ data);

    logExeInstr16(pContext, instr, "%s: MemWrite Reg %d (0x%04x) to 0x%08x - %s. Modified Bits = 0x%04x", 
        __func__, fields.t, 0xFFFF&data, address, getMemInfo(address), modified_bits);

    logExeCCode("// MemWrite %s (address was computed as reg%d + 0x%08x)\n", 
        getMemInfo(address), fields.n, (fields.imm << 1));
    logExeCCode("*(uint16_t*)0x%08x = reg%d; // = 0x%04x (modified bits = 0x%04x)\n\n", 
        address, fields.t, 0xFFFF&data, modified_bits);

    return PINKYSIM_STEP_OK;
}

static int ldrhImmediate(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeImm10to6_Rn5to3_Rt2to0(instr);
    uint32_t address;
    uint32_t data;

    address = getReg(pContext, fields.n) + (fields.imm << 1);
    data = unalignedMemRead(pContext, address, 2);
    setReg(pContext, fields.t, data);

    logExeInstr16(pContext, instr, "%s: Set Reg %d to 0x%08x (MemRead from 0x%08x - %s)", 
        __func__, fields.t, &data, address, getMemInfo(address));

    logExeCCode("// MemRead %s (address was computed as reg%d + 0x%08x)\n", 
        getMemInfo(address), fields.n, (fields.imm << 1));
    logExeCCode("reg%d = *(uint16_t*)0x%08x; // = 0x%08x\n\n", fields.t, 
        address, data);

    return PINKYSIM_STEP_OK;
}

static int strImmediateT2(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRt10to8_Imm7to0Shift2(instr);
    uint32_t n = SP;
    uint32_t address;
    uint32_t data;

    address = getReg(pContext, n) + fields.imm;
    data = getReg(pContext, fields.t);
    uint32_t orig_val = unalignedMemRead(pContext, address, 4);
    unalignedMemWrite(pContext, address, 4, data);

    uint32_t modified_bits = orig_val ^ data;

    logExeInstr16(pContext, instr, "%s: MemWrite value of Reg %d (0x%08x) to 0x%08x - %s. Modified Bits = 0x%08x", 
        __func__, fields.t, data, address, getMemInfo(address), modified_bits);

    logExeCCode("// MemWrite %s (address was computed as reg%d + 0x%08x)\n", 
        getMemInfo(address), fields.n, fields.imm);
    logExeCCode("*(uint32_t*)0x%08x = reg%d; // = 0x%08x (modified bits = 0x%08x)\n\n", 
        address, fields.t, data, modified_bits);

    return PINKYSIM_STEP_OK;
}

static int ldrImmediateT2(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRt10to8_Imm7to0Shift2(instr);
    uint32_t n = SP;
    uint32_t address;
    uint32_t value;

    address = getReg(pContext, n) + fields.imm;
    value = unalignedMemRead(pContext, address, 4);
    setReg(pContext, fields.t, value);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with value 0x%08x (MemRead address 0x%08x - %s)", 
        __func__, fields.t, value, address, getMemInfo(address));

    logExeCCode("// MemRead %s (address was computed as reg%d + 0x%08x)\n", 
        getMemInfo(address), fields.n, fields.imm);
    logExeCCode("reg%d = *(uint32_t*)0x%08x; // = 0x%08x\n\n", fields.t, 
        address, value);

    return PINKYSIM_STEP_OK;
}

static int adr(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRd10to8_Imm7to0(instr);
    uint32_t result;

    result = align(getReg(pContext, PC), 4) + (fields.imm << 2);
    setReg(pContext, fields.d, result);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with 0x%08x", 
        __func__, fields.d, result);

    logExeCCode("reg%d = 0x%08x; // = PC + 0x%08x\n\n", fields.d, 
        result, fields.imm << 2);

    return PINKYSIM_STEP_OK;
}

static int addSPT1(PinkySimContext* pContext, uint16_t instr)
{
    Fields     fields = decodeRd10to8_Imm7to0(instr);
    AddResults addResults;

    addResults = addWithCarry(getReg(pContext, SP), (fields.imm << 2), 0);
    setReg(pContext, fields.d, addResults.result);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with 0x%08x", 
        __func__, fields.d, addResults.result);

    logExeCCode("reg%d = 0x%08x; // = SP + 0x%08x\n\n", fields.d, 
        addResults.result, fields.imm << 2);

    return PINKYSIM_STEP_OK;
}

static int misc16BitInstructions(PinkySimContext* pContext, uint16_t instr)
{
    int result = PINKYSIM_STEP_UNDEFINED;

    if ((instr & 0x0F80) == 0x0000)
        result = addSPT2(pContext, instr);
    else if ((instr & 0x0F80) == 0x0080)
        result = subSP(pContext, instr);
    else if ((instr & 0x0FC0) == 0x0200)
        result = sxth(pContext, instr);
    else if ((instr & 0x0FC0) == 0x0240)
        result = sxtb(pContext, instr);
    else if ((instr & 0x0FC0) == 0x0280)
        result = uxth(pContext, instr);
    else if ((instr & 0x0FC0) == 0x02C0)
        result = uxtb(pContext, instr);
    else if ((instr & 0x0E00) == 0x0400)
        result = push(pContext, instr);
    else if ((instr & 0x0FE0) == 0x0660)
        result = cps(pContext, instr);
    else if ((instr & 0x0FC0) == 0x0A00)
        result = rev(pContext, instr);
    else if ((instr & 0x0FC0) == 0x0A40)
        result = rev16(pContext, instr);
    else if ((instr & 0x0FC0) == 0x0AC0)
        result = revsh(pContext, instr);
    else if ((instr & 0x0E00) == 0x0C00)
        result = pop(pContext, instr);
    else if ((instr & 0x0F00) == 0x0E00)
    {
        logExeInstr16(pContext, instr, "%s: Exception: bkptException", __func__);
        __throw(bkptException)
    }
    else if ((instr & 0x0F00) == 0x0F00)
        result = hints(pContext, instr);
    return result;
}

static int addSPT2(PinkySimContext* pContext, uint16_t instr)
{
    Fields     fields = decodeRdisSP_Imm6to0Shift2(instr);
    AddResults addResults;

    addResults = addWithCarry(getReg(pContext, SP), fields.imm, 0);
    setReg(pContext, fields.d, addResults.result);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with 0x%08x", 
        __func__, fields.d, addResults.result);

    logExeCCode("reg%d = 0x%08x; // = SP + 0x%08x\n\n", fields.d, 
        addResults.result, fields.imm);

    return PINKYSIM_STEP_OK;
}

static Fields decodeRdisSP_Imm6to0Shift2(uint32_t instr)
{
    Fields fields;

    fields.d = SP;
    fields.imm = (instr & 0x7F) << 2;
    return fields;
}

static int subSP(PinkySimContext* pContext, uint16_t instr)
{
    Fields     fields = decodeRdisSP_Imm6to0Shift2(instr);
    AddResults addResults;

    addResults = addWithCarry(getReg(pContext, SP), ~fields.imm, 1);
    setReg(pContext, fields.d, addResults.result);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with 0x%08x", 
        __func__, fields.d, addResults.result);

    logExeCCode("reg%d = 0x%08x; // = SP - 0x%08x\n\n", fields.d, 
        addResults.result, fields.imm);

    return PINKYSIM_STEP_OK;
}

static int sxth(PinkySimContext* pContext, uint16_t instr)
{
    Fields fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t data;

    data = signExtend16(getReg(pContext, fields.m));
    setReg(pContext, fields.d, data);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with 0x%08x", 
        __func__, fields.d, data);

    logExeCCode("reg%d = 0x%08x; // = signExtend16(fields.m)\n\n", fields.d, 
        data);

    return PINKYSIM_STEP_OK;
}

static int sxtb(PinkySimContext* pContext, uint16_t instr)
{
    Fields fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t data;

    data = signExtend8(getReg(pContext, fields.m));
    setReg(pContext, fields.d, data);

    logExeInstr16(pContext, instr, "%s: Set Reg %d with 0x%08x", 
        __func__, fields.d, data);

    logExeCCode("reg%d = 0x%08x; // = signExtend8(fields.m)\n\n", fields.d, 
        data);

    return PINKYSIM_STEP_OK;
}

static int uxth(PinkySimContext* pContext, uint16_t instr)
{
    Fields fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t data;

    data = zeroExtend16(getReg(pContext, fields.m));
    setReg(pContext, fields.d, data);

    logExeInstr16(pContext, instr, "%s: Set Reg %d to 0x%04x", 
        __func__, fields.d, data);

    logExeCCode("reg%d = 0x%08x; // = zeroExtend16(fields.m)\n\n", fields.d, 
        data);

    return PINKYSIM_STEP_OK;
}

static int uxtb(PinkySimContext* pContext, uint16_t instr)
{
    Fields fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t data;

    data = zeroExtend8(getReg(pContext, fields.m));
    setReg(pContext, fields.d, data);

    logExeInstr16(pContext, instr, "%s: Set Reg %d to 0x%02x", 
        __func__, fields.d, data);

    logExeCCode("reg%d = 0x%08x; // = zeroExtend8(fields.m)\n\n", fields.d, 
        data);

    return PINKYSIM_STEP_OK;
}

static int push(PinkySimContext* pContext, uint16_t instr)
{
    uint32_t    registers = ((instr & (1 << 8)) << 6) | (instr & 0xFF);
    uint32_t    address;
    int         i;

    if (bitCount(registers) < 1)
    {
        logExeInstr16(pContext, instr, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }

    logExeCCode("{\n");
    logExeIncIndentCCode();

    address = getReg(pContext, SP) - 4 * bitCount(registers);
    for (i = 0 ; i <= 14 ; i++)
    {
        if (registers & (1 << i))
        {
            logExeCCode("// Save reg%d to Stack at 0x%08x (Value saved is 0x%08x)\n", 
                i, address, getReg(pContext, i));
            alignedMemWrite(pContext, address, 4, getReg(pContext, i));
            address += 4;
        }
    }
    setReg(pContext, SP, getReg(pContext, SP) - 4 * bitCount(registers));
    logExeCCode("// Stack Pointer updated to 0x%08x\n\n", getReg(pContext, SP));

    logExeInstr16(pContext, instr, "%s: SP = 0x%08x", __func__, getReg(pContext, SP));

    return PINKYSIM_STEP_OK;
}

static uint32_t bitCount(uint32_t value)
{
    uint32_t count = 0;

    while (value)
    {
        value = value & (value - 1);
        count++;
    }
    return count;
}

static int cps(PinkySimContext* pContext, uint16_t instr)
{
    uint32_t im = instr & (1 << 4);

    if ((instr & 0xF) != 0x2)
    {
        logExeInstr16(pContext, instr, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }

    if (currentModeIsPrivileged(pContext))
    {
        if (im)
            pContext->PRIMASK |= PRIMASK_PM;
        else
            pContext->PRIMASK &= ~PRIMASK_PM;
    }

    logExeInstr16(pContext, instr, "%s", __func__);

    return PINKYSIM_STEP_OK;
}

static int currentModeIsPrivileged(PinkySimContext* pContext)
{
    // NOTE: This simulator only supports privileged mode.
    return TRUE;
}

static int rev(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t value;
    uint32_t result;

    value = getReg(pContext, fields.m);
    result = (value << 24) | (value >> 24) | ((value & 0xFF00) << 8) | ((value & 0xFF0000) >> 8);
    setReg(pContext, fields.d, result);

    logExeInstr16(pContext, instr, "%s: Set Reg %d to 0x%08x", __func__, 
        fields.d, result);

    logExeCCode("reg%d = 0x%08x; // = lots_o_math(field.m)\n\n", fields.d, 
        result);

    return PINKYSIM_STEP_OK;
}

static int rev16(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t value;
    uint32_t result;

    value = getReg(pContext, fields.m);
    result = ((value & 0xFF00FF00) >> 8) | ((value & 0x00FF00FF) << 8);
    setReg(pContext, fields.d, result);

    logExeInstr16(pContext, instr, "%s: Set Reg %d to 0x%08x", 
        __func__, fields.d, result);

    logExeCCode("reg%d = 0x%08x; // = lots_o_math(field.m)\n\n", fields.d, 
        result);

    return PINKYSIM_STEP_OK;
}

static int revsh(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRm5to3_Rdn2to0(instr);
    uint32_t value;
    uint32_t result;

    value = getReg(pContext, fields.m);
    result = ((value & 0xFF00FF00) >> 8) | ((value & 0x00FF00FF) << 8);
    setReg(pContext, fields.d, signExtend16(result));

    logExeInstr16(pContext, instr, "%s: Set Reg %d to 0x%08x", 
        __func__, fields.d, result);

    logExeCCode("reg%d = 0x%08x; // = lots_o_math(field.m)\n\n", fields.d, 
        result);

    return PINKYSIM_STEP_OK;
}

static int pop(PinkySimContext* pContext, uint16_t instr)
{
    uint32_t    registers = ((instr & (1 << 8)) << 7) | (instr & 0xFF);
    uint32_t    address;
    int         i;

    if (bitCount(registers) < 1)
    {
        logExeInstr16(pContext, instr, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }

    logExeDecIndentCCode();
    logExeCCode("}\n");

    address = getReg(pContext, SP);
    for (i = 0 ; i <= 7 ; i++)
    {
        if (registers & (1 << i))
        {
            setReg(pContext, i, alignedMemRead(pContext, address, 4));
            logExeCCode("// Restore reg%d from Stack at 0x%08x (Value saved was 0x%08x)\n", 
                i, address, getReg(pContext, i));
            address += 4;
        }
    }
    if (registers & (1 << 15))
    {
        loadWritePC(pContext, alignedMemRead(pContext, address, 4));
        logExeCCode("// Restore PC from Stack at 0x%08x (Value saved was 0x%08x)\n", 
            address, alignedMemRead(pContext, address, 4));
    }
    setReg(pContext, SP, getReg(pContext, SP) + 4 * bitCount(registers));
    logExeCCode("// Stack Pointer updated to 0x%08x\n\n", getReg(pContext, SP));

    logExeInstr16(pContext, instr, "%s: SP = 0x%08x", __func__, getReg(pContext, SP));

    return PINKYSIM_STEP_OK;
}

static void loadWritePC(PinkySimContext* pContext, uint32_t address)
{
    bxWritePC(pContext, address);
}

static int hints(PinkySimContext* pContext, uint16_t instr)
{
    uint32_t opA = (instr & (0x00F0)) >> 4;
    uint32_t opB = instr & 0x000F;
    int      result = PINKYSIM_STEP_UNDEFINED;

    if (opB != 0x0000)
    {
        logExeInstr16(pContext, instr, "%s: Exception: undefinedException", __func__);
        __throw(undefinedException);
    }
    switch (opA)
    {
    case 0:
        result = nop(pContext, instr);
        break;
    case 1:
        result = yield(pContext, instr);
        break;
    case 2:
        result = wfe(pContext, instr);
        break;
    case 3:
        result = wfi(pContext, instr);
        break;
    case 4:
        result = sev(pContext, instr);
        break;
    default:
        result = treatAsNop(pContext, instr);
        break;
    }
    return result;
}

static int nop(PinkySimContext* pContext, uint16_t instr)
{
    logExeInstr16(pContext, instr, "%s", __func__);

    logExeCCode("__nop\n\n");

    return PINKYSIM_STEP_OK;
}

static int yield(PinkySimContext* pContext, uint16_t instr)
{
    logExeInstr16(pContext, instr, "%s", __func__);

    logExeCCode("__yield\n\n");

    return PINKYSIM_STEP_UNSUPPORTED;
}

static int wfe(PinkySimContext* pContext, uint16_t instr)
{
    logExeInstr16(pContext, instr, "%s", __func__);

    logExeCCode("__wfe\n\n");

    return PINKYSIM_STEP_UNSUPPORTED;
}

static int wfi(PinkySimContext* pContext, uint16_t instr)
{
    logExeInstr16(pContext, instr, "%s", __func__);

    logExeCCode("__wfi\n\n");

    return PINKYSIM_STEP_UNSUPPORTED;
}

static int sev(PinkySimContext* pContext, uint16_t instr)
{
    logExeInstr16(pContext, instr, "%s", __func__);

    logExeCCode("__sev\n\n");

    return PINKYSIM_STEP_UNSUPPORTED;
}

static int treatAsNop(PinkySimContext* pContext, uint16_t instr)
{
    logExeInstr16(pContext, instr, "%s", __func__);

    logExeCCode("__treadAsNop\n\n");

    return PINKYSIM_STEP_OK;
}

static int stm(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRn10to8RegisterList7to0(instr);
    uint32_t address;
    int      i;

    if (bitCount(fields.registers) < 1)
    {
        logExeInstr16(pContext, instr, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }
    if ((fields.registers & (1 << fields.n)) && isNotLowestBitSet(fields.registers, fields.n))
    {
        logExeInstr16(pContext, instr, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }

    address = getReg(pContext, fields.n);

    logExeInstr16(pContext, instr, "%s: Start address is Reg %d (0x%08x). fields.registers = 0x%02x (for values written into mem). Reg %d += %d", 
        __func__, fields.n, address, fields.registers, fields.n, 4 * bitCount(fields.registers));

    logExeCCode("// stm (Store Multiple):\n");
 
    for (i = 0 ; i <= 14 ; i++)
    {
        if (fields.registers & (1 << i))
        {
            uint32_t orig = alignedMemRead(pContext, address, 4);
            alignedMemWrite(pContext, address, 4, getReg(pContext, i));

            uint32_t modified_bits = orig ^ getReg(pContext, i);

            logExeCCode("*(uint32_t*)0x%08x = reg%d; // 0x%08x (modified bits = 0x%08x)\n", 
                address, i, getReg(pContext, i), modified_bits);

            address += 4;
        }
    }

    logExeCCode("reg%d = reg%d + 4*bitCount(fields.registers); // = 0x%08x\n",
        fields.n, fields.n, getReg(pContext, fields.n) + 4 * bitCount(fields.registers));

    setReg(pContext, fields.n, getReg(pContext, fields.n) + 4 * bitCount(fields.registers));

    logExeCCode("\n");

    return PINKYSIM_STEP_OK;
}

static Fields decodeRn10to8RegisterList7to0(uint32_t instr)
{
    Fields fields;

    fields.n = (instr & (7 << 8)) >> 8;
    fields.registers = instr & 0xFF;
    return fields;
}

static int isNotLowestBitSet(uint32_t bits, uint32_t i)
{
    return (int)(((1 << i) - 1) & bits);
}

static int ldm(PinkySimContext* pContext, uint16_t instr)
{
    Fields   fields = decodeRn10to8RegisterList7to0(instr);
    int      wback = (0 == (fields.registers & (1 << fields.n)));
    uint32_t address;
    int      i;

    if (bitCount(fields.registers) < 1)
    {
        logExeInstr16(pContext, instr, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }

    address = getReg(pContext, fields.n);

    logExeInstr16(pContext, instr, "%s: Start address is Reg %d (0x%08x). fields.registers = 0x%02x (for loading with mem reads). Reg %d += %d if %d", 
        __func__, fields.n, address, fields.registers, fields.n, 4 * bitCount(fields.registers), wback);

    logExeCCode("// ldm (Load Multiple):\n");

    for (i = 0 ; i <= 7 ; i++)
    {
        if (fields.registers & (1 << i))
        {
            setReg(pContext, i, alignedMemRead(pContext, address, 4));

            logExeCCode("reg%d = *(uint32_t*)0x%08x; // = 0x%08x\n", i, 
                address, alignedMemRead(pContext, address, 4));

            address += 4;
        }
    }

    if (wback) 
    {
        logExeCCode("reg%d = reg%d + 4*bitCount(fields.registers); // = 0x%08x\n",
            fields.n, fields.n, getReg(pContext, fields.n) + 4 * bitCount(fields.registers));

        setReg(pContext, fields.n, getReg(pContext, fields.n) + 4 * bitCount(fields.registers));
    }

    logExeCCode("\n");

    return PINKYSIM_STEP_OK;
}

static int conditionalBranchAndSupervisor(PinkySimContext* pContext, uint16_t instr)
{
    if ((instr & 0x0F00) == 0x0E00)
    {
        logExeInstr16(pContext, instr, "%s: Exception: undefinedException", __func__);
        __throw(undefinedException)
    }
    else if ((instr & 0x0F00) == 0x0F00)
        return svc(pContext, instr);
    else
        return conditionalBranch(pContext, instr);
}

static int svc(PinkySimContext* pContext, uint16_t instr)
{
    return PINKYSIM_STEP_SVC;
}

static int conditionalBranch(PinkySimContext* pContext, uint16_t instr)
{
    int32_t imm32 = (((int32_t)(instr & 0xFF)) << 24) >> 23;
    uint32_t branchAddr = getReg(pContext, PC) + imm32;
    uint32_t cond = (instr & 0x0F00) >> 8;
    char* cond_strs[16] = 
        {
            "Equal, Z == 1",
            "Not equal, Z == 0",
            "Carry set, C == 1",
            "Carry clear, C == 0",
            "Minus, negative, N == 1",
            "Plus, positive or zero, N == 0",
            "Overflow, V == 1",
            "No overflow, V == 0",
            "Unsigned higher, C == 1 && Z == 0",
            "Unsigned lower or same, C == 0 && Z == 1",
            "Signed greater than or equal, N == V",
            "Signed less than, N != V",
            "Signed greater than, Z == 0 && N == V",
            "Signed less than or equal, Z == 1 && N != V",
            "None",
            "None",
        }; 


    if (conditionPassedForBranchInstr(pContext, instr))
    {
        logExeInstr16(pContext, instr, "%s: Branching to 0x%08x, NOT executing 0x%08x (Condition check %s)", 
            __func__, branchAddr, pContext->newPC, cond_strs[cond]);

        logExeCCode("NOT %s\n", cond_strs[cond]);
        logExeCCode("{\n");
        logExeIncIndentCCode();
        logExeCCode("// UNKOWN PATH execute 0x%08x\n", pContext->newPC);
        logExeDecIndentCCode();
        logExeCCode("}\n\n");

        branchWritePC(pContext, branchAddr);
    }
    else
    {
        logExeInstr16(pContext, instr, "%s: NOT branching to 0x%08x, executing 0x%08x (Condition check %s)", 
            __func__, branchAddr, pContext->newPC, cond_strs[cond]);

        logExeCCode("%s\n", cond_strs[cond]);
        logExeCCode("{\n");
        logExeIncIndentCCode();
        logExeCCode("// UNKOWN PATH execute 0x%08x\n", branchAddr);
        logExeDecIndentCCode();
        logExeCCode("}\n\n");
    }

    return PINKYSIM_STEP_OK;
}

static int conditionPassedForBranchInstr(PinkySimContext* pContext, uint16_t instr)
{
    uint32_t cond = (instr & 0x0F00) >> 8;
    uint32_t apsr = pContext->xPSR;
    int      result = FALSE;

    switch (cond >> 1)
    {
    case 0:
        result = ((apsr & APSR_Z) == APSR_Z);
        break;
    case 1:
        result = ((apsr & APSR_C) == APSR_C);
        break;
    case 2:
        result = ((apsr & APSR_N) == APSR_N);
        break;
    case 3:
        result = ((apsr & APSR_V) == APSR_V);
        break;
    case 4:
        result = ((apsr & APSR_C) == APSR_C) && ((apsr & APSR_Z) == 0);
        break;
    case 5:
        result = (!!(apsr & APSR_N) == !!(apsr & APSR_V));
        break;
    case 6:
        result = ((!!(apsr & APSR_N) == !!(apsr & APSR_V)) && ((apsr & APSR_Z) == 0));
        break;
    // NOTE: Can't be used as a condition code in ARMv6-M
    //case 7:
    //    result = TRUE;
    }
    if ((cond & 1) && (cond != 0xF))
        result = !result;
    return result;
}

static int unconditionalBranch(PinkySimContext* pContext, uint16_t instr)
{
    int32_t imm32 = (((int32_t)(instr & 0x7FF)) << 21) >> 20;

    branchWritePC(pContext, getReg(pContext, PC) + imm32);

    logExeInstr16(pContext, instr, "%s: Branch to 0x%08x", 
        __func__, pContext->newPC);

    logExeCCode("// Branching from PC = 0x%08x to PC = 0x%08x\n\n", 
        pContext->pc, pContext->newPC);

    return PINKYSIM_STEP_OK;
}

static int executeInstruction32(PinkySimContext* pContext, uint16_t instr1)
{
    int      result = PINKYSIM_STEP_UNDEFINED;
    uint16_t instr2;

    pContext->newPC = pContext->pc + 4;
    instr2 =  IMemory_Read16(pContext->pMemory, pContext->pc + 2);

    if ((instr1 & 0x1800) == 0x1000 && (instr2 & 0x8000) == 0x8000)
        result = branchAndMiscellaneousControl(pContext, instr1, instr2);
    return result;
}

static int branchAndMiscellaneousControl(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2)
{
    if ((instr2 & 0x5000) == 0x0000 && (instr1 & 0x07E0) == 0x0380)
        return msr(pContext, instr1, instr2);
    else if ((instr2 & 0x5000) == 0x0000 && (instr1 & 0x07F0) == 0x03B0)
        return miscellaneousControl(pContext, instr1, instr2);
    else if ((instr2 & 0x5000) == 0x0000 && (instr1 & 0x07E0) == 0x03E0)
        return mrs(pContext, instr1, instr2);
    else if ((instr2 & 0x5000) == 0x5000)
        return bl(pContext, instr1, instr2);

    logExeInstr32(pContext, instr1, instr2, "%s: Exception: undefinedException", __func__);
    __throw(undefinedException);
}

static int msr(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2)
{
    uint32_t n = instr1 & 0xF;
    uint32_t SYSm = instr2 & 0xFF;
    uint32_t value;

    if (n == 13 || n == 15)
    {
        logExeInstr32(pContext, instr1, instr2, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }
    if (SYSm == 4 || (SYSm > 9 && SYSm < 16) || (SYSm > 16 && SYSm < 20) || (SYSm > 20))
    {
        logExeInstr32(pContext, instr1, instr2, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }
    if ((instr1 & 0x0010) != 0x0000 || (instr2 & 0x3F00) != 0x0800)
    {
        logExeInstr32(pContext, instr1, instr2, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }

    value = getReg(pContext, n);
    switch (SYSm >> 3)
    {
    case 0:
        if ((SYSm & (1 << 2)) == 0)
            pContext->xPSR = (pContext->xPSR & ~APSR_NZCV) | (value & APSR_NZCV);
        break;
    case 1:
        if (currentModeIsPrivileged(pContext))
        {
            switch (SYSm & 0x7)
            {
            case 0:
                pContext->spMain = value & 0xFFFFFFFC;
                break;
            case 1:
                // NOTE: This simulator doesn't support process stack usage.
                break;
            }
        }
        break;
    case 2:
        if (currentModeIsPrivileged(pContext))
        {
            switch (SYSm & 0x7)
            {
            case 0:
                pContext->PRIMASK = (pContext->PRIMASK & ~PRIMASK_PM) | (value & PRIMASK_PM);
                break;
            case 4:
                // NOTE: This simulator doesn't support thread mode.
                break;
            }
        }
        break;
    }

    logExeInstr32(pContext, instr1, instr2, "%s", __func__);

    logExeCCode("// msr\n\n");

    return PINKYSIM_STEP_OK;
}

static int miscellaneousControl(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2)
{
    int      result = PINKYSIM_STEP_UNDEFINED;

    if ((instr2 & 0x00F0) == 0x0040)
        result = dsb(pContext, instr1, instr2);
    else if ((instr2 & 0x00F0) == 0x0050)
        result = dmb(pContext, instr1, instr2);
    else if ((instr2 & 0x00F0) == 0x0060)
        result = isb(pContext, instr1, instr2);
    return result;
}

static int dsb(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2)
{
    if ((instr1 & 0x000F) != 0x000F || (instr2 & 0x2F00) != 0x0F00)
    {
        logExeInstr32(pContext, instr1, instr2, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }

    logExeInstr32(pContext, instr1, instr2, "%s", __func__);

    logExeCCode("// dsb\n\n");

    return PINKYSIM_STEP_OK;
}

static int dmb(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2)
{
    if ((instr1 & 0x000F) != 0x000F || (instr2 & 0x2F00) != 0x0F00)
    {
        logExeInstr32(pContext, instr1, instr2, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }

    logExeInstr32(pContext, instr1, instr2, "%s", __func__);

    logExeCCode("// dmb\n\n");

    return PINKYSIM_STEP_OK;
}

static int isb(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2)
{
    if ((instr1 & 0x000F) != 0x000F || (instr2 & 0x2F00) != 0x0F00)
    {
        logExeInstr32(pContext, instr1, instr2, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }

    logExeInstr32(pContext, instr1, instr2, "%s", __func__);

    logExeCCode("// isb\n\n");

    return PINKYSIM_STEP_OK;
}

static int mrs(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2)
{
    uint32_t d = (instr2 & (0xF << 8)) >> 8;
    uint32_t SYSm = instr2 & 0xFF;
    uint32_t value = 0;

    if (d == 13 || d == 15)
    {
        logExeInstr32(pContext, instr1, instr2, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }
    if (SYSm == 4 || (SYSm > 9 && SYSm < 16) || (SYSm > 16 && SYSm < 20) || (SYSm > 20))
    {
        logExeInstr32(pContext, instr1, instr2, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }
    if ((instr1 & 0x001F) != 0x000F || (instr2 & 0x2000) != 0x0000)
    {
        logExeInstr32(pContext, instr1, instr2, "%s: Exception: unpredictableException", __func__);
        __throw(unpredictableException);
    }

    switch (SYSm >> 3)
    {
    case 0:
        if ((SYSm & (1 << 0)) && currentModeIsPrivileged(pContext))
            value |= pContext->xPSR & IPSR_MASK;
        if ((SYSm & (1 << 1)))
            value |= 0; /* T-bit reads as zero on ARMv-6m */
        if ((SYSm & (1 << 2)) == 0)
            value |= pContext->xPSR & APSR_NZCV;
        break;
    case 1:
        if (currentModeIsPrivileged(pContext))
        {
            switch (SYSm & 0x7)
            {
            case 0:
                value = pContext->spMain;
                break;
            case 1:
                // NOTE: This simulator doesn't support process stack usage.
                break;
            }
        }
        break;
    case 2:
        switch (SYSm & 0x7)
        {
        case 0:
            value = pContext->PRIMASK & PRIMASK_PM;
            break;
        case 4:
            value = pContext->CONTROL;
            break;
        }
        break;
    }
    setReg(pContext, d, value);

    logExeInstr32(pContext, instr1, instr2, "%s", __func__);

    logExeCCode("// mrs\n\n");

    return PINKYSIM_STEP_OK;
}

static int bl(PinkySimContext* pContext, uint16_t instr1, uint16_t instr2)
{
    uint32_t s = (instr1 & (1 << 10)) >> 10;
    uint32_t immHi = instr1 & 0x3FF;
    uint32_t j1 = (instr2 & (1 << 13)) >> 13;
    uint32_t j2 = (instr2 & (1 << 11)) >> 11;
    uint32_t immLo = (instr2 & 0x7FF);
    uint32_t i1 = (s ^ j1) ^ 1;
    uint32_t i2 = (s ^ j2) ^ 1;
    uint32_t imm32 = ((int32_t)((s << 24) | (i1 << 23) | (i2 << 22) | (immHi << 12) | (immLo << 1)) << 7) >> 7;
    uint32_t nextInstrAddr;
    uint32_t branchAddr;

    nextInstrAddr = getReg(pContext, PC);
    setReg(pContext, LR, nextInstrAddr | 1);
    branchAddr = getReg(pContext, PC) + imm32;
    branchWritePC(pContext, branchAddr);

    logExeInstr32(pContext, instr1, instr2, "%s: Branch to 0x%08x (Link Reg set to 0x%08x)", 
        __func__, branchAddr, nextInstrAddr | 1);

    logExeCCode("// Branch from 0x%08x to 0x%08x (Set LR to 0x%08x)\n\n",
        pContext->pc, branchAddr, nextInstrAddr | 1);

    return PINKYSIM_STEP_OK;
}
