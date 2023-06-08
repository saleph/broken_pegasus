#include "src/C6202.hpp"

#include <algorithm>
#include <tuple>
#include <stdexcept>
#include <string>
#include <format>


C6202::C6202(IClock& clock, const RAM& memory)
    : clock{clock}, memory{memory} {
    resetRegisters();
}

C6202::C6202(IClock &clock, const Program &program) 
    : clock{clock}, memory{} {
    resetRegisters();
    memory = RAM{program.getProgram()};
}

void C6202::resetRegisters()
{
    reg = Registers{};
}

Registers C6202::getRegisters() const
{
    return reg;
}

const RAM& C6202::getMemory() const {
    return memory;
}

void C6202::run() {
    resetRegisters();
    clock.start();
    while (!isBreakInstructionMet()) {
        runNextInstruction();
    }
}

bool C6202::isBreakInstructionMet() const {
    return memory[reg.PC] == 0x00;
}

void C6202::tick() {
    clock.waitForNextTick();
}

void C6202::runNextInstruction() {
    // https://llx.com/Neil/a2/opcodes.html
    const auto opcode = memory[reg.PC];
    tick();
    // opcode: aaabbbcc The aaa and cc bits determine the opcode, and the bbb bits determine the addressing mode. 
    const auto instructionGroupMask = uint8_t{0b00000011u};
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
}

void C6202::runGroupOneInstruction(const uint8_t opcode) {
}

void C6202::runGroupTwoInstruction(const uint8_t opcode) {
}

void C6202::runGroupThreeInstruction(const uint8_t opcode) {
}