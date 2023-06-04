#include "src/RAM.hpp"

#include <cassert>

RAM::RAM(const std::array<std::byte, RAM::SIZE>& memory) 
    : memory{memory} {
}

std::byte& RAM::operator[](const size_t address) {
    assert(idx < SIZE && "Address out of range of RAM");
    return memory[address];
}