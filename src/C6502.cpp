#include "src/C6502.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <tuple>
#include <stdexcept>
#include <string>
#include <format>
#include "C6502.hpp"

static const bool DONT_CARE_ABOUT_PAGE_BOUNDARY_CROSSING = false;

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
    spdlog::trace("Starting CPU");
    resetRegisters();
    clock.start();
    while (runNextInstruction()) {
        ;
    }
}

void C6502::tick() {
    clock.waitForNextTick();
}

uint8_t& C6502::accessMemory(const uint16_t address) {
    tick();
    return memory[address];
}

bool C6502::runNextInstruction() {
    // https://llx.com/Neil/a2/opcodes.html
    const auto opcode = accessMemory(reg.PC);
    ++reg.PC;
    const auto isBrkInstruction = opcode == 0x00u;
    if (isBrkInstruction) {
        spdlog::trace("BRK met");
        return false;
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
    return true;
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
    return result & signBit;
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
    const auto [operand, didCrossPageBoundary] = getValueFrom(addressingResult);
    if (didCrossPageBoundary) {
        tick();
    }
    const auto substractionResult = reg.A - operand;
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
    spdlog::trace("Group three instruction: {:x}", opcode);
}