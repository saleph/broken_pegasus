#ifndef SRC_C6202_HPP
#define SRC_C6202_HPP

#include "src/RAM.hpp"
#include "src/IClock.hpp"
#include "src/Program.hpp"

#include <array>
#include <bitset>
#include <cstdint>
#include <tuple>
#include <variant>

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

/// @brief CPU definition. I used mostly https://llx.com/Neil/a2/opcodes.html
class C6502 {
    public:
    C6502(IClock& clock, const RAM& memory);
    C6502(IClock& clock, const Program& program);

    /// @brief It will run till first BRK instruction (0x00) is met
    void run();

    Registers getRegisters() const;
    const RAM& getMemory() const;
    
    private:
    struct MemoryAddress {
        uint16_t address;
        bool didCrossPageBoundary;
    };
    enum class ImmediateValue : uint8_t {};
    using AddressingResult = std::variant<MemoryAddress, ImmediateValue>;

    // Registers
    // https://en.wikipedia.org/wiki/MOS_Technology_6502#Registers
    IClock& clock;
    Registers reg = {};
    RAM memory;

    static uint8_t getInstuctionGroup(const uint8_t opcode);
    static uint8_t getAddressingMode(const uint8_t opcode);
    static uint8_t getInstructionType(const uint8_t opcode);
    void resetRegisters();
    /// @brief ticks are happening when interaction with memory occurs.
    /// Also, when memory boundary is crossed when indexing (difference on bits 8-15): 
    /// idx>>8 ^ (idx+X)>>8 > 0
    /// one additional tick is performed, but only for READ instructions
    /// (write instructions perform this tick always, to avoid a write into wrong location)
    void tick();
    uint8_t& accessMemory(const uint16_t address);
    uint16_t readTwoByteAddressAtLocation(const uint16_t address);
    uint16_t concatAddress(const uint16_t lowHalf, const uint16_t highHalf) const;
    bool runNextInstruction();
    void runGroupOneInstruction(const uint8_t opcode);
    AddressingResult getAddressingIndirectZeroPageX();
    AddressingResult getAddressingZeroPage();
    AddressingResult getAddressingImmediate();
    AddressingResult getAddressingAbsolute();
    AddressingResult getAddressingIndirectZeroPageY();
    AddressingResult getAddressingZeroPageX();
    AddressingResult getAddressingAbsoluteXY(const uint8_t xOrYRegister);
    void runORA(const AddressingResult addressingResult);
    void runAND(const AddressingResult addressingResult);
    void runEOR(const AddressingResult addressingResult);
    void runADC(const AddressingResult addressingResult);
    void runSTA(const AddressingResult addressingResult);
    void runLDA(const AddressingResult addressingResult);
    void runCMP(const AddressingResult addressingResult);
    void runSBC(const AddressingResult addressingResult);
    bool doesCrossPageBoundary(const uint16_t baseAddress, const uint16_t indexedAddress) const;
    void runGroupTwoInstruction(const uint8_t opcode);
    void runGroupThreeInstruction(const uint8_t opcode);
};

#endif  // SRC_C6202_HPP
