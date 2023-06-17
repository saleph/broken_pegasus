#ifndef SRC_RAM_HPP
#define SRC_RAM_HPP

#include "src/Program.hpp"

#include <array>
#include <vector>
#include <cstdint>

class RAM {
    public:
    static constexpr size_t SIZE = 1 << 16;
    RAM() = default;
    RAM(const RAM& other) = default;
    RAM(const Program& program);
    RAM(const std::array<uint8_t, SIZE>& memory);

    uint8_t& operator[](const size_t address);
    const uint8_t& operator[](const size_t address) const;
    bool operator==(const RAM& other) const = default;
    auto begin() noexcept { return memory.begin(); }
    auto end() noexcept { return memory.end(); }
    auto cbegin() const noexcept { return memory.cbegin(); }
    auto cend() const noexcept { return memory.cend(); }
    const std::array<uint8_t, SIZE>& getUnderlayingArray() const { return memory; }

    private:
    static constexpr size_t CODE_ORIGIN_LOCATION = 0x0600;
    std::array<uint8_t, SIZE> memory;
};

#endif  // SRC_RAM_HPP
