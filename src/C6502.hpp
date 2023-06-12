#ifndef SRC_C6502_HPP
#define SRC_C6502_HPP

#include "src/RAM.hpp"
#include "src/IClock.hpp"
#include "src/Program.hpp"

#include <array>
#include <bitset>
#include <cstdint>
#include <tuple>
#include <variant>

struct Registers {
    /// @brief accumulator
    uint8_t A = 0x0;
    /// @brief X index
    uint8_t X = 0x0;
    /// @brief Y index
    uint8_t Y = 0x0;
    /// @brief stack pointer
    uint8_t SP = 0xff;
    /// @brief program counter
    uint16_t PC = 0x0600;
    union {
        struct {
            /// @brief carry
            bool C : 1;
            /// @brief zero
            bool Z : 1;
            /// @brief interrupt disable
            bool I : 1;
            /// @brief decimal mode
            bool D : 1;
            /// @brief break
            bool B : 1;
            bool __reserved : 1;
            /// @brief overflow
            bool V : 1;
            /// @brief negative
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
    enum class MemoryAddress : uint16_t {};
    enum class ImmediateValue : uint8_t {};
    using ValueOrAddress = std::variant<MemoryAddress, ImmediateValue>;
    struct AddressingResult {
        ValueOrAddress valueOrAddress;
        bool didCrossPageBoundary;
    };
    struct DataAndCrossPageBoundariesCrossing {
        uint8_t value;
        bool didCrossPageBoundary;
    };

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
    DataAndCrossPageBoundariesCrossing getValueFrom(const AddressingResult addressingResult);
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
    bool getZeroFlag(const uint8_t result) const;
    bool getNegativeSignBit(const uint8_t result) const;
    bool getCarryFlag(const int result) const;
    bool getOverflowFlag(const uint8_t accumulatorBeforeOperation, const int result) const;
    void performDecimalADC(const uint8_t operand);
    void runGroupTwoInstruction(const uint8_t opcode);
    void runGroupThreeInstruction(const uint8_t opcode);
};

#endif  // SRC_C6502_HPP
