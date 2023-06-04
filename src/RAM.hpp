#ifndef SRC_RAM_HPP
#define SRC_RAM_HPP

#include <array>
#include <cstddef>

class RAM {
    public:
    static constexpr size_t SIZE = 1 << 16;
    RAM() = default;
    RAM(const std::array<std::byte, SIZE>& memory);
    std::byte& operator[](const size_t address);
    
    private:
    std::array<std::byte, SIZE> memory;
};

#endif  // SRC_RAM_HPP
