#ifndef SRC_C6202_HPP
#define SRC_C6202_HPP

#include "src/RAM.hpp"
#include "src/IClock.hpp"

#include <array>
#include <bitset>
#include <cstddef>

class C6202 {
    public:
    C6202(IClock& clock, const RAM& memory);
    void start();
    
    private:
    // Registers
    // https://en.wikipedia.org/wiki/MOS_Technology_6502#Registers
    IClock& clock;
    uint8_t A;
    uint8_t X;
    uint8_t Y;
    uint8_t SP;
    uint16_t PC;
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


    void resetRegisters();
};

#endif  // SRC_C6202_HPP
