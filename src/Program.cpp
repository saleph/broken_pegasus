#include "src/Program.hpp"

#include <regex>

Program::Program(const std::vector<uint8_t>& program)
    : program{program} {
}

Program::Program(const std::string& hexdump) {
    std::regex hexRegex("[0-9a-f]{2}", std::regex_constants::ECMAScript | std::regex_constants::icase);
    for (auto i = std::sregex_iterator(hexdump.begin(), hexdump.end(), hexRegex); 
            i != std::sregex_iterator(); 
            ++i) {
        const auto match = std::smatch{*i};
        const auto matchStr = match.str();
        const auto parsedValue = std::stoul(matchStr, nullptr, 16);
        program.emplace_back(parsedValue);
    }
}
