#ifndef SRC_ICLOCK_HPP
#define SRC_ICLOCK_HPP

class IClock {
    public:
    virtual ~IClock() = default;
    virtual void start() = 0;
    virtual void waitForNextTick() = 0;
};

#endif  // SRC_ICLOCK_HPP
