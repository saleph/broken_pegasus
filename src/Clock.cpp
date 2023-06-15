#include "src/Clock.hpp"

#include "src/Clock.hpp"

#include <limits>
#include <iostream>
#include "Clock.hpp"

using namespace std::chrono_literals;

Clock::Clock(WaiterFunction waiterFunction, const size_t frequencyHz) 
    : waiter{std::move(waiterFunction)} {
    setFrequency(frequencyHz);
}

void Clock::start() {
    lastTickTime = std::chrono::steady_clock::now();
}

void Clock::waitForNextTick() {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - lastTickTime);
    waiter(period - elapsed);
    lastTickTime += period;
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
