#ifndef SRC_C6202_HPP
#define SRC_C6202_HPP

#include "src/RAM.hpp"

#include <array>
#include <bitset>
#include <cstddef>

class C6202 {
    public:
    
    private:
    // Registers
    // https://en.wikipedia.org/wiki/MOS_Technology_6502#Registers
    uint8_t A = 0x0;
    uint8_t X = 0x0;
    uint8_t Y = 0x0;
    uint8_t SP = 0xff;
    uint16_t PC = 0x0600;
    union {
        struct {
            bool C : 1;
            bool Z : 1;
            bool I : 1;
            bool D : 1;
            bool B : 1;
            bool __reserved : 1;
            bool V : 1;
            bool N : 1;
        };
        std::bitset<8> flags;
    };
    RAM memory;
};

#endif  // SRC_C6202_HPP
