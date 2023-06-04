#ifndef SRC_CLOCK_HPP
#define SRC_CLOCK_HPP

#include "src/IClock.hpp"

#include <chrono>
#include <cstddef>
#include <functional>
#include <thread>

class Clock : public IClock {
    public:
    using WaiterFunction = std::function<void(std::chrono::nanoseconds)>;
    
    Clock(WaiterFunction waiterFunction = [](auto ns){ std::this_thread::sleep_for(ns); });
    ~Clock() override = default;
    void start() override;
    void waitForNextTick() override;

    void setFrequency(const size_t frequencyHz);
    size_t getFrequencyHz() const;

    private:
    std::chrono::nanoseconds period;
    std::chrono::time_point<std::chrono::steady_clock> lastTickTime;
    WaiterFunction waiter;
};

#endif  // SRC_CLOCK_HPP
