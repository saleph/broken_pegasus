#include "src/C6502.hpp"

#include <algorithm>
#include <tuple>
#include <stdexcept>
#include <string>
#include <format>

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
    resetRegisters();
    clock.start();
    while (!runNextInstruction()) {
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
        return false;
    }
    switch (opcode & instructionGroupMask) {
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
    switch (opcode & addresingModeMask) {
        case 0b000:	addressingResult = getAddressingIndirectZeroPageX(); break;
        case 0b001:	addressingResult = getAddressingZeroPage(); break;
        case 0b010:	addressingResult = getAddressingImmediate(); break;
        case 0b011:	addressingResult = getAddressingAbsolute(); break;
        case 0b100:	addressingResult = getAddressingIndirectZeroPageY(); break;
        case 0b101:	addressingResult = getAddressingZeroPageX(); break;
        case 0b110:	addressingResult = getAddressingAbsoluteY(); break;
        case 0b111:	addressingResult = getAddressingAbsoluteX(); break;
    }
    switch (opcode & instructionTypeMask) {
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

// 000	(zero page,X)
// 001	zero page
// 010	#immediate
// 011	absolute
// 100	(zero page),Y
// 101	zero page,X
// 110	absolute,Y
// 111	absolute,X

C6502::AddressingResult C6502::getAddressingIndirectZeroPageX() {
    // (zero page,X)
    const auto zeroPageBaseAddress = accessMemory(reg.PC++);
    const auto discardedUnindexedZeroPageAddressData = accessMemory(zeroPageBaseAddress);
    (void)discardedUnindexedZeroPageAddressData;
    //TODO: potentially here can be another tick(), but I'm not sure yet
    const auto zeroPageEffectiveAddress = static_cast<uint16_t>(zeroPageBaseAddress + reg.X);
    tick();
    const auto indirectAddress = readTwoByteAddressAtLocation(zeroPageEffectiveAddress);
    const auto effectiveAddress = readTwoByteAddressAtLocation(indirectAddress);
    return MemoryAddress{effectiveAddress, DONT_CARE_ABOUT_PAGE_BOUNDARY_CROSSING};
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
    // zero page
    const auto effectiveAddress = accessMemory(reg.PC++);
    return MemoryAddress{effectiveAddress, DONT_CARE_ABOUT_PAGE_BOUNDARY_CROSSING};
}

C6502::AddressingResult C6502::getAddressingImmediate() {
    // #immediate
    const auto immediateValue = accessMemory(reg.PC++);
    return ImmediateValue{immediateValue};
}

C6502::AddressingResult C6502::getAddressingAbsolute() {
    // absolute
    const auto lowHalfAddress = accessMemory(reg.PC++);
    const auto highHalfAddress = accessMemory(reg.PC++);
    const auto effectiveAddress = concatAddress(lowHalfAddress, highHalfAddress);
    return MemoryAddress{effectiveAddress, DONT_CARE_ABOUT_PAGE_BOUNDARY_CROSSING};
}

C6502::AddressingResult C6502::getAddressingIndirectZeroPageY() {
    // (zero page),Y
    const auto indirectAddressLocationAtZeroPage = accessMemory(reg.PC++);
    const auto indirectAddress = readTwoByteAddressAtLocation(indirectAddressLocationAtZeroPage);
    const auto indexedIndirectAddress = indirectAddress + reg.Y;
    const auto effectiveAddress = readTwoByteAddressAtLocation(indexedIndirectAddress);
    return MemoryAddress{effectiveAddress, doesCrossPageBoundary(indirectAddress, indexedIndirectAddress + 1)};
}

bool C6502::doesCrossPageBoundary(const uint16_t baseAddress, const uint16_t indexedAddress) const {
    const auto baseAddressHighHalf = baseAddress >> 8;
    const auto indexedAddressHighHalf = indexedAddress >> 8;
    return (baseAddressHighHalf ^ indexedAddressHighHalf) > 0;
}

C6502::AddressingResult C6502::getAddressingZeroPageX() {
    // zero page,X
}

C6502::AddressingResult C6502::getAddressingAbsoluteY() {
    // absolute,Y
}

C6502::AddressingResult C6502::getAddressingAbsoluteX() {
    // absolute,X
}

void C6502::runORA(const AddressingResult addressingResult) {

}

void C6502::runAND(const AddressingResult addressingResult) {

}

void C6502::runEOR(const AddressingResult addressingResult) {

}

void C6502::runADC(const AddressingResult addressingResult) {

}

void C6502::runSTA(const AddressingResult addressingResult) {

}

void C6502::runLDA(const AddressingResult addressingResult) {

}

void C6502::runCMP(const AddressingResult addressingResult) {

}

void C6502::runSBC(const AddressingResult addressingResult) {

}

void C6502::runGroupTwoInstruction(const uint8_t opcode) {
}

void C6502::runGroupThreeInstruction(const uint8_t opcode) {
}