#include "src/C6202.hpp"

#include <algorithm>
#include <tuple>


C6202::C6202(IClock& clock, const RAM& memory)
    : clock{clock}, memory{memory} {
    resetRegisters();
}

C6202::C6202(IClock &clock, const Program &program) 
    : clock{clock}, memory{} {
    resetRegisters();
    memory = RAM{program.getProgram()};
}

void C6202::resetRegisters() {
    reg = Registers{};
}

void C6202::run() {
    while (true) {
        const auto b = memory[reg.PC++];
        if (b == 0u) {
            return;
        }
    }
}

Registers C6202::getRegisters() const {
    return reg;
}

RAM C6202::getMemory() const {
    return memory;
}
