#include "src/C6502.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <tuple>
#include <stdexcept>
#include <string>
#include <format>
#include "C6502.hpp"

static const bool DONT_CARE_ABOUT_PAGE_BOUNDARY_CROSSING = false;
const auto STACK_PAGE = 0b0001'0000u;

C6502::C6502(IClock& clock, const RAM& memory)
    : clock{clock}, memory{memory} {
    resetRegisters();
}

C6502::C6502(IClock &clock, const Program &program) 
    : clock{clock}, memory{} {
    resetRegisters();
    memory = RAM{program.getProgram()};
}

// opcode: aaabbbcc The aaa and cc bits determine the opcode, 
// and the bbb bits determine the addressing mode. cc determines the group
uint8_t C6502::getInstuctionGroup(const uint8_t opcode) {
    static const uint8_t instructionGroupMask = 0b00000011u;
    return opcode & instructionGroupMask;
}

uint8_t C6502::getAddressingMode(const uint8_t opcode) {
    static const uint8_t addresingModeMask = 0b00011100u;
    return (opcode & addresingModeMask) >> 2;
}

uint8_t C6502::getInstructionType(const uint8_t opcode) {
    static const uint8_t instructionTypeMask = 0b11100000u;
    return (opcode & instructionTypeMask) >> 5;
}

void C6502::resetRegisters() {
    reg = Registers{};
}

Registers C6502::getRegisters() const {
    return reg;
}

const RAM& C6502::getMemory() const {
    return memory;
}

void C6502::run() {
    spdlog::debug("Starting CPU");
    clock.start();
    while (!reg.B) {
        runNextInstruction();
    }
}

void C6502::tick() {
    clock.waitForNextTick();
}

uint8_t& C6502::accessMemory(const uint16_t address) {
    tick();
    return memory[address];
}

void C6502::runNextInstruction() {
    // https://llx.com/Neil/a2/opcodes.html
    const auto opcode = accessMemory(reg.PC);
    ++reg.PC;
    switch (opcode) {
        case 0x00: runBRK(); return;
        case 0x20: runJSR(); return;
        case 0x40: runRTI(); return;
        case 0x60: runRTS(); return; 
        // single byte instructions
        case 0x08: runPHP(); return;
        case 0x28: runPLP(); return;
        case 0x48: runPHA(); return;
        case 0x68: runPLA(); return;
        case 0x88: runDEY(); return;
        case 0xA8: runTAY(); return;
        case 0xC8: runINY(); return;
        case 0xE8: runINX(); return;
        case 0x18: runCLC(); return;
        case 0x38: runSEC(); return;
        case 0x58: runCLI(); return;
        case 0x78: runSEI(); return;
        case 0x98: runTYA(); return;
        case 0xB8: runCLV(); return;
        case 0xD8: runCLD(); return;
        case 0xF8: runSED(); return;
        case 0x8A: runTXA(); return;
        case 0x9A: runTXS(); return;
        case 0xAA: runTAX(); return;
        case 0xBA: runTSX(); return;
        case 0xCA: runDEX(); return;
        case 0xEA: runNOP(); return;
        default:
            break;
    }
    if ((opcode & 0b0001'1111u) == 0b0001'0000u) {
        runBranchInstruction(opcode);
        return;
    }
    switch (getInstuctionGroup(opcode)) {
        case 0b01: runGroupOneInstruction(opcode); break;
        case 0b10: runGroupTwoInstruction(opcode); break;
        case 0b00: runGroupThreeInstruction(opcode); break;
        case 0b11:
        default: 
            throw std::runtime_error(std::format(
                "Unsupported opcode: {:#02x}/{:#08b} from group 0b00000011 "
                "(potentially 65816 instruction)", opcode, opcode));
            break;
    }
}

void C6502::runBRK() {
    // TODO
    spdlog::trace("BRK");
    reg.B = true;
}

void C6502::runJSR() {
    spdlog::trace("JSR");
    const auto newAddressLow = accessMemory(reg.PC++);
    tick(); // store ADL
    const auto oldAddressHigh = static_cast<uint8_t>((reg.PC & 0xFF00u) >> 8);
    pushOnStack(oldAddressHigh);
    const auto oldAddressLow = static_cast<uint8_t>(reg.PC & 0x00FFu);
    pushOnStack(oldAddressLow);
    const auto newAddressHigh = accessMemory(reg.PC++);
    reg.PC = concatAddress(newAddressLow, newAddressHigh);
}

void C6502::pushOnStack(const uint8_t value) {
    const auto address = STACK_PAGE | reg.SP;
    accessMemory(address) = value;
    --reg.SP;
}

uint8_t C6502::popFromStack() {
    ++reg.SP;
    const auto address = STACK_PAGE | reg.SP;
    return accessMemory(address);
}

void C6502::runRTI() {
    spdlog::trace("RTI");
    const auto discarded = accessMemory(reg.PC);
    (void)discarded;
    const auto discardedStackByte = accessMemory(STACK_PAGE | reg.SP);
    (void)discardedStackByte;
    const auto newFlags = popFromStack();
    const auto newPCLow = popFromStack();
    const auto newPCHigh = popFromStack();
    reg.PC = concatAddress(newPCLow, newPCHigh);
    reg.flags = newFlags;
}

void C6502::runRTS() {
    spdlog::trace("RTS");
    const auto discarded1 = accessMemory(reg.PC++);
    (void)discarded1;
    const auto discarded2 = accessMemory(STACK_PAGE | reg.SP);
    (void)discarded2;
    const auto newPCLow = popFromStack();
    const auto newPCHigh = popFromStack();
    const auto discarded3 = accessMemory(reg.PC++);
    (void)discarded3;
    reg.PC = concatAddress(newPCLow, newPCHigh);
    // fix PC after (-1) store during JSR
    ++reg.PC;
}

void C6502::runPHP() {
    spdlog::trace("PHP");
    const auto discarded1 = accessMemory(reg.PC);
    (void)discarded1;
    pushOnStack(reg.flags);
}

void C6502::runPLP() {
    spdlog::trace("PLP");
    const auto discarded1 = accessMemory(reg.PC);
    (void)discarded1;
    reg.flags = popFromStack();
    tick();
}

void C6502::runPHA() {
    spdlog::trace("PHA");
    const auto discarded1 = accessMemory(reg.PC);
    (void)discarded1;
    pushOnStack(reg.A);
}

void C6502::runPLA() {
    spdlog::trace("PLA");
    const auto discarded1 = accessMemory(reg.PC);
    (void)discarded1;
    const auto value = popFromStack();
    reg.A = value;
    tick();
    reg.N = getNegativeSignBit(reg.A);
    reg.Z = getZeroFlag(reg.A);
}

void C6502::runDEY() {
    spdlog::trace("DEY");
    decrementRegister(reg.Y);
}

void C6502::runTAY() {
    spdlog::trace("TAY");
    transferRegister(reg.A, reg.Y);
}

void C6502::runINY() {
    spdlog::trace("INY");
    incrementRegister(reg.Y);
}

void C6502::incrementRegister(uint8_t& reg) {
    tick();
    ++reg;
}

void C6502::decrementRegister(uint8_t& reg) {
    tick();
    --reg;
}

void C6502::runINX() {
    spdlog::trace("INX");
    incrementRegister(reg.X);
}

void C6502::runCLC() {
    spdlog::trace("CLC");
    tick();
    reg.C = false;
}

void C6502::runSEC() {
    spdlog::trace("SEC");
    tick();
    reg.C = true;
}

void C6502::runCLI() {
    spdlog::trace("CLI");
    tick();
    reg.I = false;
}

void C6502::runSEI() {
    spdlog::trace("SEI");
    tick();
    reg.I = true;
}

void C6502::runTYA() {
    spdlog::trace("TYA");
    transferRegister(reg.Y, reg.A);
}

void C6502::runCLV() {
    spdlog::trace("CLV");
    tick();
    reg.V = false;
}

void C6502::runCLD() {
    spdlog::trace("CLD");
    tick();
    reg.D = false;
}

void C6502::runSED() {
    spdlog::trace("SED");
    tick();
    reg.D = true;
}

void C6502::runTXA() {
    spdlog::trace("TXA");
    transferRegister(reg.X, reg.A);
}

void C6502::runTXS() {
    spdlog::trace("TXS");
    transferRegister(reg.X, reg.SP);
}

void C6502::runTAX() {
    spdlog::trace("TAX");
    transferRegister(reg.A, reg.X);
}

void C6502::transferRegister(const uint8_t source, uint8_t& destination) {
    const auto discarded1 = accessMemory(reg.PC);
    (void)discarded1;
    destination = source;
    reg.N = getNegativeSignBit(destination);
    reg.Z = getZeroFlag(destination);
}

void C6502::runTSX() {
    spdlog::trace("TSX");
    transferRegister(reg.SP, reg.X);
}

void C6502::runDEX() {
    spdlog::trace("DEX");
    decrementRegister(reg.X);
}

void C6502::runNOP() {
    spdlog::trace("NOP");
    tick();
}

void C6502::runBranchInstruction(const uint8_t opcode) {
    // The conditional branch instructions all have the form xxy10000. The flag
    // indicated by xx is compared with y, and the branch is taken if they are equal.
    const auto xxFlagType = (opcode & 0b1100'0000) >> 6;
    const auto yExpectedValue = static_cast<bool>((opcode & 0b0010'0000) >> 5);
    switch (xxFlagType) {
        case 0b00:
            spdlog::trace("B{}", yExpectedValue ? "MI" : "PL");
            runConditionalJump(reg.N, yExpectedValue); 
            break; // negative
        case 0b01:
            spdlog::trace("BV{}", yExpectedValue ? "S" : "C");
            runConditionalJump(reg.V, yExpectedValue); 
            break; // overflow
        case 0b10:
            spdlog::trace("BC{}", yExpectedValue ? "S" : "C");
            runConditionalJump(reg.C, yExpectedValue); 
            break; // carry
        case 0b11:
            spdlog::trace("B{}", yExpectedValue ? "EQ" : "NE");
            runConditionalJump(reg.Z, yExpectedValue); 
            break; // zero
    }
}

void C6502::runConditionalJump(const bool flag, const bool expectedValueOfTheFlag) {
    spdlog::trace("Conditional branch: flag={}, expected={}", flag, expectedValueOfTheFlag);
    const auto offset = accessMemory(reg.PC);
    ++reg.PC;
    if (flag != expectedValueOfTheFlag) {
        return;
    }
    const auto newPC = static_cast<uint16_t>(reg.PC + offset);
    const auto pageNumberMask = uint16_t{0xFF00u};
    const auto oldPCPageNumber = reg.PC & pageNumberMask;
    const auto newPCPageNumber = newPC & pageNumberMask;
    if (oldPCPageNumber != newPCPageNumber) {
        // page boundary crossing
        tick();
    }
    reg.PC = newPC;
}

void C6502::runGroupOneInstruction(const uint8_t opcode) {
    AddressingResult addressingResult;
    switch (getAddressingMode(opcode)) {
        case 0b000: addressingResult = getAddressingIndirectZeroPageX(); break;
        case 0b001: addressingResult = getAddressingZeroPage(); break;
        case 0b010: addressingResult = getAddressingImmediate(); break;
        case 0b011: addressingResult = getAddressingAbsolute(); break;
        case 0b100: addressingResult = getAddressingIndirectZeroPageY(); break;
        case 0b101: addressingResult = getAddressingZeroPageX(); break;
        case 0b110: addressingResult = getAddressingAbsoluteXY(reg.Y); break;
        case 0b111: addressingResult = getAddressingAbsoluteXY(reg.X); break;
    }
    switch (getInstructionType(opcode)) {
        case 0b000: runORA(addressingResult); break;
        case 0b001: runAND(addressingResult); break;
        case 0b010: runEOR(addressingResult); break;
        case 0b011: runADC(addressingResult); break;
        case 0b100: runSTA(addressingResult); break;
        case 0b101: runLDA(addressingResult); break;
        case 0b110: runCMP(addressingResult); break;
        case 0b111: runSBC(addressingResult); break;
    }
}

C6502::AddressingResult C6502::getAddressingIndirectZeroPageX() {
    spdlog::trace("zero page,X)");
    const auto zeroPageBaseAddress = accessMemory(reg.PC++);
    const auto discardedUnindexedZeroPageAddressData = accessMemory(zeroPageBaseAddress);
    (void)discardedUnindexedZeroPageAddressData;
    //TODO: potentially here can be another tick(), but I'm not sure yet
    const auto zeroPageEffectiveAddress = static_cast<uint16_t>(zeroPageBaseAddress + reg.X);
    tick();
    const auto indirectAddress = readTwoByteAddressAtLocation(zeroPageEffectiveAddress);
    const auto effectiveAddress = readTwoByteAddressAtLocation(indirectAddress);
    return {MemoryAddress{effectiveAddress}, DONT_CARE_ABOUT_PAGE_BOUNDARY_CROSSING};
}

uint16_t C6502::readTwoByteAddressAtLocation(const uint16_t address) {
    const auto lowHalf = accessMemory(address);
    const auto highHalf = accessMemory(address + 1);
    return concatAddress(lowHalf, highHalf);
}

uint16_t C6502::concatAddress(const uint16_t lowHalf, const uint16_t highHalf) const {
    return static_cast<uint16_t>(lowHalf | (highHalf << 8));
}

C6502::AddressingResult C6502::getAddressingZeroPage() {
    spdlog::trace("zero page");
    const auto effectiveAddress = accessMemory(reg.PC++);
    return {MemoryAddress{effectiveAddress}, DONT_CARE_ABOUT_PAGE_BOUNDARY_CROSSING};
}

C6502::AddressingResult C6502::getAddressingImmediate() {
    spdlog::trace("#immediate");
    const auto immediateValue = accessMemory(reg.PC++);
    return {ImmediateValue{immediateValue}, DONT_CARE_ABOUT_PAGE_BOUNDARY_CROSSING};
}

C6502::AddressingResult C6502::getAddressingAbsolute() {
    spdlog::trace("absolute");
    const auto lowHalfAddress = accessMemory(reg.PC++);
    const auto highHalfAddress = accessMemory(reg.PC++);
    const auto effectiveAddress = concatAddress(lowHalfAddress, highHalfAddress);
    return {MemoryAddress{effectiveAddress}, DONT_CARE_ABOUT_PAGE_BOUNDARY_CROSSING};
}

C6502::AddressingResult C6502::getAddressingIndirectZeroPageY() {
    spdlog::trace("(zero page),Y");
    const auto indirectAddressLocationAtZeroPage = accessMemory(reg.PC++);
    const auto indirectAddress = readTwoByteAddressAtLocation(indirectAddressLocationAtZeroPage);
    const auto indexedIndirectAddress = indirectAddress + reg.Y;
    const auto effectiveAddress = readTwoByteAddressAtLocation(indexedIndirectAddress);
    return {MemoryAddress{effectiveAddress}, doesCrossPageBoundary(indirectAddress, indexedIndirectAddress + 1)};
}

bool C6502::doesCrossPageBoundary(const uint16_t baseAddress, const uint16_t indexedAddress) const {
    const auto baseAddressHighHalf = baseAddress >> 8;
    const auto indexedAddressHighHalf = indexedAddress >> 8;
    return (baseAddressHighHalf ^ indexedAddressHighHalf) > 0;
}

C6502::AddressingResult C6502::getAddressingZeroPageX() {
    spdlog::trace("zero page,X");
    const auto zeroPageBaseAddress = accessMemory(reg.PC++);
    const auto discardedUnindexedZeroPageAddressData = accessMemory(zeroPageBaseAddress);
    (void)discardedUnindexedZeroPageAddressData;

    const auto wrappedAroundIndexedAddressOnPageZero = static_cast<uint8_t>(zeroPageBaseAddress + reg.X);
    const auto effectiveAddress = wrappedAroundIndexedAddressOnPageZero;
    return {MemoryAddress{effectiveAddress}, DONT_CARE_ABOUT_PAGE_BOUNDARY_CROSSING};
}

C6502::AddressingResult C6502::getAddressingAbsoluteXY(const uint8_t xOrYRegister) {
    spdlog::trace("absolute,X/Y");
    const auto lowHalfAddress = accessMemory(reg.PC++);
    const auto highHalfAddress = accessMemory(reg.PC++);
    const auto baseAddress = concatAddress(lowHalfAddress, highHalfAddress);
    const auto indexedEffectiveAddress = static_cast<uint16_t>(baseAddress + xOrYRegister);
    return {MemoryAddress{indexedEffectiveAddress}, doesCrossPageBoundary(baseAddress, indexedEffectiveAddress + 1)};
}

void C6502::runORA(const AddressingResult addressingResult) {
    spdlog::trace("ORA");
    const auto [operand, didCrossPageBoundary] = getValueFrom(addressingResult);
    if (didCrossPageBoundary) {
        tick();
    }
    reg.A |= operand;
    reg.N = getNegativeSignBit(reg.A);
    reg.Z = getZeroFlag(reg.A);
}

C6502::DataAndCrossPageBoundariesCrossing C6502::getValueFrom(const AddressingResult addressingResult) {
    if (const auto memoryAddress = std::get_if<MemoryAddress>(&addressingResult.valueOrAddress); memoryAddress) {
        return {memory[static_cast<uint16_t>(*memoryAddress)], addressingResult.didCrossPageBoundary};
    }
    if (const auto value = std::get_if<ImmediateValue>(&addressingResult.valueOrAddress); value) {
        return {static_cast<uint8_t>(*value), addressingResult.didCrossPageBoundary};
    }
    throw std::invalid_argument("Unexpected type of AddressingResult");
}

bool C6502::getZeroFlag(const uint8_t result) const {
    return result == 0;
}

bool C6502::getNegativeSignBit(const uint8_t result) const {
    const auto signBit = uint8_t{0b1000'0000};
    return (result & signBit) >> 7;
}

void C6502::runAND(const AddressingResult addressingResult) {
    spdlog::trace("AND");
    const auto [operand, didCrossPageBoundary] = getValueFrom(addressingResult);
    if (didCrossPageBoundary) {
        tick();
    }
    reg.A &= operand;
    reg.N = getNegativeSignBit(reg.A);
    reg.Z = getZeroFlag(reg.A);
}

void C6502::runEOR(const AddressingResult addressingResult) {
    spdlog::trace("EOR");
    const auto [operand, didCrossPageBoundary] = getValueFrom(addressingResult);
    if (didCrossPageBoundary) {
        tick();
    }
    reg.A ^= operand;
    reg.N = getNegativeSignBit(reg.A);
    reg.Z = getZeroFlag(reg.A);
}

void C6502::runADC(const AddressingResult addressingResult) {
    spdlog::trace("ADC");
    const auto [operand, didCrossPageBoundary] = getValueFrom(addressingResult);
    if (didCrossPageBoundary) {
        tick();
    }
    const auto oldA = reg.A;
    auto fullResult = operand + reg.A + reg.C;
    if (reg.D) {
        fullResult = normalizeBDCResult(fullResult);
    }
    const auto trimmedResult = static_cast<uint8_t>(fullResult);
    reg.A = trimmedResult;
    reg.V = getOverflowFlag(oldA, fullResult);
    reg.C = getCarryFlag(fullResult);
    reg.N = getNegativeSignBit(trimmedResult);
    reg.Z = getZeroFlag(trimmedResult);
}

int C6502::normalizeBDCResult(const int notNormalizedResult) const {
    const auto lowerDigitMask = 0b0000'1111;
    const auto higherDigitOffset = 4;
    const auto higherDigitMask = lowerDigitMask << higherDigitOffset;
    const auto lowerDigit = notNormalizedResult & lowerDigitMask;
    auto higherDigit = notNormalizedResult & higherDigitMask;
    auto normalizedResult = 0;
    if (lowerDigit < 10) {
        normalizedResult += lowerDigit;
    } else {
        normalizedResult += lowerDigit - 10;
        higherDigit += 1 << higherDigitOffset;
    }
    if (higherDigit < 10) {
        normalizedResult += higherDigit;
    } else {
        normalizedResult += higherDigit - (10 << higherDigitOffset);
        normalizedResult += 1 << (higherDigitOffset * 2);
    }
    return normalizedResult;
}

bool C6502::getCarryFlag(const int result) const {
    return result > std::numeric_limits<uint8_t>().max();
}

bool C6502::getOverflowFlag(const uint8_t accumulatorBeforeOperation, const int result) const {
    return getNegativeSignBit(accumulatorBeforeOperation) ^ getNegativeSignBit(result);
}

void C6502::runSTA(const AddressingResult addressingResult) {
    spdlog::trace("STA");
    const auto address = std::get<MemoryAddress>(addressingResult.valueOrAddress);
    memory[static_cast<uint16_t>(address)] = reg.A;
    tick();
}

void C6502::runLDA(const AddressingResult addressingResult) {
    spdlog::trace("LDA");
    const auto [operand, didCrossPageBoundary] = getValueFrom(addressingResult);
    if (didCrossPageBoundary) {
        tick();
    }
    reg.A = operand;
    reg.N = getNegativeSignBit(reg.A);
    reg.Z = getZeroFlag(reg.A);
}

void C6502::runCMP(const AddressingResult addressingResult) {
    spdlog::trace("CMP");
    compareRegister(reg.A, addressingResult);
}

void C6502::compareRegister(const uint8_t regValue, const AddressingResult addressingResult) {
    const auto [operand, didCrossPageBoundary] = getValueFrom(addressingResult);
    if (didCrossPageBoundary) {
        tick();
    }
    const auto substractionResult = regValue - operand;
    reg.N = getNegativeSignBit(substractionResult);
    reg.Z = getZeroFlag(substractionResult);
    reg.C = getCarryFlag(substractionResult);
}

void C6502::runSBC(const AddressingResult addressingResult) {
    spdlog::trace("SBC");
    const auto [operand, didCrossPageBoundary] = getValueFrom(addressingResult);
    if (didCrossPageBoundary) {
        tick();
    }
    const auto oldA = reg.A;
    auto fullResult = reg.A + static_cast<uint8_t>(~operand) + reg.C;
    if (reg.D) {
        fullResult = normalizeBDCResult(fullResult);
    }
    const auto trimmedResult = static_cast<uint8_t>(fullResult);
    reg.A = trimmedResult;
    reg.V = getOverflowFlag(oldA, fullResult);
    reg.C = getCarryFlag(fullResult);
    reg.N = getNegativeSignBit(trimmedResult);
    reg.Z = getZeroFlag(trimmedResult);
}

void C6502::runGroupTwoInstruction(const uint8_t opcode) {
    AddressingResult addressingResult;
    switch (getAddressingMode(opcode)) {
        case 0b000: addressingResult = getAddressingImmediate(); break;
        case 0b001: addressingResult = getAddressingZeroPage(); break;
        case 0b010: addressingResult = getAddressingAccumulator(); break;
        case 0b011: addressingResult = getAddressingAbsolute(); break;
        case 0b101: addressingResult = getAddressingZeroPageX(); break;
        case 0b111: addressingResult = getAddressingAbsoluteXY(reg.X); break;
        case 0b100:
        case 0b110:
            throw std::runtime_error(std::format("Wrong addressing mode: 0b{:03b} for group two instruction", getAddressingMode(opcode)));
            break;
    }
    switch (getInstructionType(opcode)) {
        case 0b000: runASL(addressingResult); break;
        case 0b001: runROL(addressingResult); break;
        case 0b010: runLSR(addressingResult); break;
        case 0b011: runROR(addressingResult); break;
        case 0b100: runSTX(addressingResult); break;
        case 0b101: runLDX(addressingResult); break;
        case 0b110: runDEC(addressingResult); break;
        case 0b111: runINC(addressingResult); break;
    }
}

C6502::AddressingResult C6502::getAddressingAccumulator() {
    spdlog::trace("accumulator");
    return {UseAccumulator{}, DONT_CARE_ABOUT_PAGE_BOUNDARY_CROSSING};
}

void C6502::runASL(const AddressingResult addressingResult) {
    spdlog::trace("ASL");
    auto& operandee = getAccumulatorOrMemoryReference(addressingResult);
    reg.C = static_cast<bool>((operandee & 0b1000'0000) >> 7);
    operandee <<= 1;
    reg.N = getNegativeSignBit(operandee);
    reg.Z = getZeroFlag(operandee);
}

uint8_t& C6502::getAccumulatorOrMemoryReference(const AddressingResult addressingResult) {
    if (const auto value = std::get_if<UseAccumulator>(&addressingResult.valueOrAddress); value) {
        tick(); // discarded byte read right after opcode
        return reg.A;
    }
    if (const auto value = std::get_if<MemoryAddress>(&addressingResult.valueOrAddress); value) {
        return accessMemory(static_cast<uint16_t>(*value));
    }
    throw std::invalid_argument("Unexpected type of AddressingResult");
}

void C6502::runROL(const AddressingResult addressingResult) {
    spdlog::trace("ROL");
    auto& operandee = getAccumulatorOrMemoryReference(addressingResult);
    const auto bit7 = static_cast<uint8_t>((operandee & 0b1000'0000) >> 7);
    operandee = (operandee << 1) | bit7;
    reg.N = getNegativeSignBit(operandee);
    reg.Z = getZeroFlag(operandee);
}

void C6502::runLSR(const AddressingResult addressingResult) {
    spdlog::trace("LSR");
    auto& operandee = getAccumulatorOrMemoryReference(addressingResult);
    reg.C = static_cast<bool>(operandee & 0b0000'0001);
    operandee >>= 1;
    reg.N = getNegativeSignBit(operandee);
    reg.Z = getZeroFlag(operandee);
}

void C6502::runROR(const AddressingResult addressingResult) {
    spdlog::trace("ROR");
    auto& operandee = getAccumulatorOrMemoryReference(addressingResult);
    const auto bit0 = static_cast<uint8_t>(operandee & 0b0000'0001);
    operandee = (operandee >> 1) | (reg.C << 7);
    reg.C = bit0;
    reg.N = getNegativeSignBit(operandee);
    reg.Z = getZeroFlag(operandee);
}

void C6502::runSTX(const AddressingResult addressingResult) {
    spdlog::trace("STX");
    auto& operandee = getAccumulatorOrMemoryReference(addressingResult);
    operandee = reg.X;
}

void C6502::runLDX(const AddressingResult addressingResult) {
    spdlog::trace("LDX");
    const auto [operand, didCrossPageBoundary] = getValueFrom(addressingResult);
    if (didCrossPageBoundary) {
        tick();
    }
    reg.X = operand;
    reg.N = getNegativeSignBit(operand);
    reg.Z = getZeroFlag(operand);
}

void C6502::runDEC(const AddressingResult addressingResult) {
    spdlog::trace("DEC");
    auto& operandee = getAccumulatorOrMemoryReference(addressingResult);
    --operandee;
    tick();
    reg.N = getNegativeSignBit(operandee);
    reg.Z = getZeroFlag(operandee);
}

void C6502::runINC(const AddressingResult addressingResult) {
    spdlog::trace("INC");
    auto& operandee = getAccumulatorOrMemoryReference(addressingResult);
    ++operandee;
    tick();
    reg.N = getNegativeSignBit(operandee);
    reg.Z = getZeroFlag(operandee);
}

void C6502::runGroupThreeInstruction(const uint8_t opcode) {
    AddressingResult addressingResult;
    switch (getAddressingMode(opcode)) {
        case 0b000: addressingResult = getAddressingImmediate(); break;
        case 0b001: addressingResult = getAddressingZeroPage(); break;
        case 0b011: addressingResult = getAddressingAbsolute(); break;
        case 0b101: addressingResult = getAddressingZeroPageX(); break;
        case 0b111: addressingResult = getAddressingAbsoluteXY(reg.X); break;
        case 0b010:
        case 0b100:
        case 0b110:
            throw std::runtime_error(std::format("Wrong addressing mode: 0b{:03b} for group three instruction", getAddressingMode(opcode)));
            break;
    }
    switch (getInstructionType(opcode)) {
        case 0b001: runBIT(addressingResult); break;
        case 0b010: runJMP(std::get<MemoryAddress>(addressingResult.valueOrAddress)); break;
        case 0b011: runJMPabsIndirect(addressingResult); break;
        case 0b100: runSTY(addressingResult); break;
        case 0b101: runLDY(addressingResult); break;
        case 0b110: runCPY(addressingResult); break;
        case 0b111: runCPX(addressingResult); break;
        case 0b000:
            throw std::runtime_error(std::format("Unknown instruction: 0b{:03b} for group three", getInstructionType(opcode)));
            break;
    }
}

void C6502::runBIT(const AddressingResult addressingResult) {
    spdlog::trace("BIT");
    const auto [operand, didCrossPageBoundary] = getValueFrom(addressingResult);
    (void)didCrossPageBoundary;
    reg.Z = getZeroFlag(operand & reg.A);
    reg.N = getNegativeSignBit(operand);
    reg.V = (operand & 0b0100'0000) >> 6;
}

void C6502::runJMP(const MemoryAddress address) {
    spdlog::trace("JMP to {:#04x}", static_cast<uint16_t>(address));
    reg.PC = static_cast<uint16_t>(address);
}

void C6502::runJMPabsIndirect(const AddressingResult addressingResult) {
    spdlog::trace("JMP indirect");
    const auto indirectAddress = static_cast<uint16_t>(std::get<MemoryAddress>(addressingResult.valueOrAddress));
    auto address = uint16_t{};
    if ((indirectAddress & 0x00FFu) != 0x00FFu) {
        address = readTwoByteAddressAtLocation(indirectAddress);
    } else {
        // http://6502.org/tutorials/6502opcodes.html#JMP
        // if address $3000 contains $40, $30FF contains $80, and $3100 contains $50, the result of JMP ($30FF)
        // will be a transfer of control to $4080 rather than $5080 as you intended i.e. the 6502 took the low 
        // byte of the address from $30FF and the high byte from $3000. 
        const auto lowHalf = accessMemory(indirectAddress);
        const auto firstAddressOfThePage = indirectAddress & 0xFF00u;
        const auto highHalf = accessMemory(firstAddressOfThePage);
        address = concatAddress(lowHalf, highHalf);
    }
    runJMP(MemoryAddress{address});
}

void C6502::runSTY(const AddressingResult addressingResult) {
    spdlog::trace("STY");
    auto& operandee = getAccumulatorOrMemoryReference(addressingResult);
    operandee = reg.Y;
}

void C6502::runLDY(const AddressingResult addressingResult) {
    spdlog::trace("LDY");
    const auto [operand, didCrossPageBoundary] = getValueFrom(addressingResult);
    if (didCrossPageBoundary) {
        tick();
    }
    reg.Y = operand;
    reg.N = getNegativeSignBit(operand);
    reg.Z = getZeroFlag(operand);
}

void C6502::runCPY(const AddressingResult addressingResult) {
    spdlog::trace("CPY");
    compareRegister(reg.Y, addressingResult);
}

void C6502::runCPX(const AddressingResult addressingResult) {
    spdlog::trace("CPX");
    compareRegister(reg.X, addressingResult);
}
