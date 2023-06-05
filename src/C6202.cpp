#include "src/C6202.hpp"

#include <algorithm>
#include "C6202.hpp"

C6202::C6202(IClock& clock, const RAM& memory)
    : clock{clock}, memory{memory} {
    resetRegisters();
}

void C6202::resetRegisters() {
    A = 0x0;
    X = 0x0;
    Y = 0x0;
    SP = 0xff;
    PC = 0x0600;
    C = false;
    Z = false;
    I = false;
    D = false;
    B = false;
    __reserved = true;
    V = false;
    N = false;
}

void C6202::start() {

}