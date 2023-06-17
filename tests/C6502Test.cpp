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
    EXPECT_EQ(expectedRegs, cpu.getRegisters());
}
