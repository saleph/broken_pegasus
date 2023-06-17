#include <gtest/gtest.h>
#include <vector>
#include <chrono>

TEST(StupidTests, negationResult) {
    ASSERT_EQ(uint8_t{0b1111'1111}, static_cast<uint8_t>(~uint8_t{0b0000'0000}));
}

TEST(StupidTests, negativeChrono) {
    ASSERT_EQ(-5, std::chrono::nanoseconds{-5}.count());
}

TEST(StupidTests, shiftRightUint8) {
    const auto v = uint8_t{0b1000'0000};
    ASSERT_EQ(0b0100'0000, v >> 1);
}
