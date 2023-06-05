#ifndef SRC_RAM_HPP
#define SRC_RAM_HPP

#include <array>
#include <vector>
#include <cstddef>

class RAM {
    public:
    static constexpr size_t SIZE = 1 << 16;
    RAM(const std::vector<std::byte>& program = {});
    RAM(const std::array<std::byte, SIZE>& memory);

    std::byte& operator[](const size_t address);
    auto begin() noexcept { return memory.begin(); }
    auto end() noexcept { return memory.end(); }
    auto cbegin() const noexcept { return memory.cbegin(); }
    auto cend() const noexcept { return memory.cend(); }

    private:
    static constexpr size_t CODE_ORIGIN_LOCATION = 0x0600;
    std::array<std::byte, SIZE> memory;
};

#endif  // SRC_RAM_HPP
