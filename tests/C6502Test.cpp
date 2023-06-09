#include "src/C6502.hpp"
#include "src/RAM.hpp"
#include "src/Clock.hpp"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <spdlog/spdlog.h>
#include <iostream>
#include <format>

class C6202TestFixture : public testing::Test {
    public:
    Clock clock{[this](auto ns){ waitFor(ns); }};
    unsigned int ticks = 0u;

    void SetUp() override {
        spdlog::set_level(spdlog::level::trace);
    }

    void TearDown() override {
        spdlog::set_level(spdlog::level::info);
    }

    private:
    void waitFor(std::chrono::nanoseconds ns) {
        ++ticks;
        spdlog::info("tick {:02}", ticks);
    }
};

MATCHER_P(RamMatcher, otherRam, "") {
    const auto ram = arg;
    auto theSame = true;
    for (auto i = 0u; i < RAM::SIZE; ++i) {
        if (ram[i] != otherRam[i]) {
            std::cout << std::format("Difference at [{:04x}]: Expected={:02x}, Is={:02x}", i, ram[i], otherRam[i]) << "\n";
            theSame = false;
        } 
    }
    return theSame;
}

void PrintTo(const Registers& r, std::ostream* os) {
    *os << std::format("A={:02x} X={:02x} Y={:02x} SP={:04x} PC={:04x} "
        "Flags: C={:01x} Z={:01x} I={:01x} D={:01x} B={:01x} V={:01x} N={:01x}",
        r.A, r.X, r.Y, r.SP, r.PC, r.C, r.Z, r.I, r.D, r.B, r.V, r.N);
}

// LDA #$01 # 2 ticks
// STA $0200 # 4 ticks
// LDA #$05 # 2 ticks
// STA $0201 # 4 ticks
// LDA #$08 # 2 ticks
// STA $0202 # 4 ticks
// read 1 byte 0x00 # 1 tick
TEST_F(C6202TestFixture, shouldSimpleProgramWorkProperly) {
    const auto program = Program{"a9 01 8d 00 02 a9 05 8d 01 02 a9 08 8d 02 02"};
    const auto memory = RAM{program};
    auto cpu = C6502{clock, memory};
    cpu.run();

    auto expectedMemory = memory;
    expectedMemory[0x0200] = 0x01u;
    expectedMemory[0x0201] = 0x05u;
    expectedMemory[0x0202] = 0x08u;
    EXPECT_THAT(expectedMemory, RamMatcher(cpu.getMemory()));

    Registers expectedRegs;
    expectedRegs.A = 0x08u;
    expectedRegs.PC = 0x0610u;
    expectedRegs.B = true;
    EXPECT_EQ(expectedRegs, cpu.getRegisters());
    EXPECT_EQ(19u, ticks);
}

// LDA #$01
// STA $0200
TEST_F(C6202TestFixture, shouldAbsoluteAddressingWorkCorrectly) {
    const auto program = Program{"a9 01 8d 00 02"};
    const auto memory = RAM{program};
    auto cpu = C6502{clock, memory};
    cpu.run();

    auto expectedMemory = memory;
    expectedMemory[0x0200] = 0x01u;
    EXPECT_THAT(expectedMemory, RamMatcher(cpu.getMemory()));

    Registers expectedRegs;
    expectedRegs.A = 0x01u;
    expectedRegs.PC = 0x0606u;
    expectedRegs.B = true;
    EXPECT_EQ(expectedRegs, cpu.getRegisters());
}

// LDA #$01
// STA $02
TEST_F(C6202TestFixture, shouldZeroPageAddressingWorkCorrectly) {
    const auto program = Program{"a9 01 85 02"};
    const auto memory = RAM{program};
    auto cpu = C6502{clock, memory};
    cpu.run();

    auto expectedMemory = memory;
    expectedMemory[0x02] = 0x01u;
    EXPECT_THAT(expectedMemory, RamMatcher(cpu.getMemory()));

    Registers expectedRegs;
    expectedRegs.A = 0x01u;
    expectedRegs.PC = 0x0605u;
    expectedRegs.B = true;
    EXPECT_EQ(expectedRegs, cpu.getRegisters());
}

// LDX #$02
// LDA #$aa  ;A is $aa
// STA $fe,X ;Store the value of A at memory location $00
TEST_F(C6202TestFixture, shouldZeroPageIndexedAddressingWorkCorrectly) {
    const auto program = Program{"a2 02 a9 aa 95 fe"};
    const auto memory = RAM{program};
    auto cpu = C6502{clock, memory};
    cpu.run();

    auto expectedMemory = memory;
    expectedMemory[0x00] = 0xAAu;
    EXPECT_THAT(expectedMemory, RamMatcher(cpu.getMemory()));

    Registers expectedRegs;
    expectedRegs.A = 0xAAu;
    expectedRegs.X = 0x02;
    expectedRegs.PC = 0x0607u;
    expectedRegs.B = true;
    expectedRegs.N = true;
    EXPECT_EQ(expectedRegs, cpu.getRegisters());
}

