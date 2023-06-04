#ifndef SRC_CLOCK_HPP
#define SRC_CLOCK_HPP

#include "src/IClock.hpp"

#include <chrono>
#include <cstddef>

class Clock : public IClock {
    public:
    ~Clock() override = default;
    void waitForNextTick() override;
    void setFrequency(const size_t frequencyHz);
    size_t getFrequencyHz() const;

    private:
    std::chrono::nanoseconds period;
};

#endif  // SRC_CLOCK_HPP
