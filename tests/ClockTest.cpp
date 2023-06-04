#include "src/Clock.hpp"

#include <gtest/gtest.h>
#include <iostream>


class ClockTestFixture : public testing::Test {
    public:
    Clock clock;
};

TEST_F(ClockTestFixture, frequency1khzShouldBeSetCorrectly) {
    const auto khz1 = size_t{1000};
    clock.setFrequency(khz1);
    EXPECT_EQ(khz1, clock.getFrequencyHz());
}

TEST_F(ClockTestFixture, frequency2mhzShouldBeSetCorrectly) {
    const auto mhz2 = size_t{2000000};
    clock.setFrequency(mhz2);
    EXPECT_EQ(mhz2, clock.getFrequencyHz());
}
