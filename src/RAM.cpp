#include "src/RAM.hpp"

#include <cassert>
#include <algorithm>

RAM::RAM(const Program& program) {
    const auto& hexdump = program.getProgram();
    assert(hexdump.size() > (SIZE - CODE_ORIGIN_LOCATION) && "Program leaks outside of available RAM");
    std::copy(hexdump.begin(), hexdump.end(), memory.begin() + CODE_ORIGIN_LOCATION);
}

RAM::RAM(const std::array<uint8_t, RAM::SIZE> &memory)
    : memory{memory} {
}

uint8_t& RAM::operator[](const size_t address) {
    assert(idx < SIZE && "Address out of range of RAM");
    return memory[address];
}

const uint8_t& RAM::operator[](const size_t address) const {
    assert(idx < SIZE && "Address out of range of RAM");
    return memory[address];
}