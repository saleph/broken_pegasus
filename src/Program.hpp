#ifndef SRC_PROGRAM_HPP
#define SRC_PROGRAM_HPP

#include <vector>
#include <string>
#include <cstdint>

class Program {
    public:
    Program(const std::vector<uint8_t>& program = {});
    Program(const std::string& hexdump);

    const std::vector<uint8_t>& getProgram() const { return program; }

    private:
    std::vector<uint8_t> program;
};

#endif  // SRC_PROGRAM_HPP
