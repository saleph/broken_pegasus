#include "src/Clock.hpp"

#include "src/Clock.hpp"

#include <limits>
#include <iostream>

using namespace std::chrono_literals;

void Clock::waitForNextTick() {
}

void Clock::setFrequency(const size_t frequencyHz) {
    if (frequencyHz == std::numeric_limits<size_t>::max()) {
        period = 0ns;
        return;
    }
    period = std::chrono::nanoseconds{std::nano::den / frequencyHz};
}

size_t Clock::getFrequencyHz() const {
    if (period == 0ns) {
        return std::numeric_limits<size_t>::max();
    }
    return std::nano::den / period.count();
}
