#include <gtest/gtest.h>
#include <vector>

#include "src/Program.hpp"

TEST(ProgramTest, shouldHexdumpWithSpacesBeParsedProperly) {
    const auto hexdump = std::string{"0f 0A 23"};
    const auto parsedHexdump = std::vector{uint8_t{0x0fu}, uint8_t{0x0au}, uint8_t{0x23u}};
    Program pr{hexdump};
    ASSERT_EQ(parsedHexdump, pr.getProgram());
}