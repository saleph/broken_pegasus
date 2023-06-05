#include "src/RAM.hpp"

#include <cassert>
#include <algorithm>

RAM::RAM(const std::vector<std::byte> &program) {
    assert(program.size() > (SIZE - CODE_ORIGIN_LOCATION) && "Program leaks outside of available RAM");
    std::copy(program.begin(), program.end(), memory.begin() + CODE_ORIGIN_LOCATION);
}

RAM::RAM(const std::array<std::byte, RAM::SIZE> &memory)
    : memory{memory} {
}

std::byte& RAM::operator[](const size_t address) {
    assert(idx < SIZE && "Address out of range of RAM");
    return memory[address];
}