#include "gcae.hpp"
#include <iostream>
#include <sstream>
#include <string>

int main()
{
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    while (true)
    {
        std::string line;
        std::string fullInput;

        while (std::getline(std::cin, line))
        {
            if (line == "END")
                break;

            fullInput += line + "\n";
        }

        if (fullInput.empty())
            break;

        std::istringstream iss(fullInput);

        EngineResult result = best_move_from_stream(iss);

        std::cout << result.move.type << ' '
                  << result.move.torow << ' '
                  << result.move.tocol << ' '
                  << result.score << ' '
                  << result.winChance << std::endl;
    }

    return 0;
}
