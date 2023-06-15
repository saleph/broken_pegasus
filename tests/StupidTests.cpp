#include <gtest/gtest.h>
#include <vector>

TEST(StupidTests, negationResult) {
    ASSERT_EQ(uint8_t{0b1111'1111}, static_cast<uint8_t>(~uint8_t{0b0000'0000}));
}