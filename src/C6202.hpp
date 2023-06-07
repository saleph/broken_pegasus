#ifndef SRC_C6202_HPP
#define SRC_C6202_HPP

#include "src/RAM.hpp"
#include "src/IClock.hpp"
#include "src/Program.hpp"

#include <array>
#include <bitset>
#include <cstddef>
#include <tuple>

struct Registers {
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
        std::bitset<8> flags = 0b00100000;
    };

    constexpr bool operator==(const Registers &o) const {
        return std::tie(A, X, Y, SP, PC, flags) == std::tie(o.A, o.X, o.Y, o.SP, o.PC, o.flags);
    }
};

class C6202 {
    public:
    C6202(IClock& clock, const RAM& memory);
    C6202(IClock& clock, const Program& program);
    void run();
    Registers getRegisters() const;
    RAM getMemory() const;
    
    private:
    // Registers
    // https://en.wikipedia.org/wiki/MOS_Technology_6502#Registers
    IClock& clock;
    Registers reg = {};
    RAM memory;


    void resetRegisters();
};

#endif  // SRC_C6202_HPP
