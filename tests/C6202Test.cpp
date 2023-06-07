#include "src/C6202.hpp"
#include "src/RAM.hpp"
#include "src/Clock.hpp"

#include <gtest/gtest.h>
#include <iostream>


class C6202TestFixture : public testing::Test {
    public:
    Clock clock{[this](auto ns){ waitFor(ns); }};

    private:
    void waitFor(std::chrono::nanoseconds ns) {
    }
};

// LDA #$01
// STA $0200
// LDA #$05
// STA $0201
// LDA #$08
// STA $0202
TEST_F(C6202TestFixture, shouldSimpleProgramWorkProperly) {
    const auto program = Program{"a9 01 8d 00 02 a9 05 8d 01 02 a9 08 8d 02 02"};
    const auto memory = RAM{program};
    auto cpu = C6202{clock, memory};
    cpu.run();

    auto expectedMemory = memory;
    expectedMemory[0x0200] = 0x01u;
    expectedMemory[0x0201] = 0x05u;
    EXPECT_EQ(expectedMemory, cpu.getMemory());

    Registers expectedRegs;
    expectedRegs.A = 0x08u;
    expectedRegs.PC = 0x0612u;
    EXPECT_EQ(expectedRegs, cpu.getRegisters());
}
