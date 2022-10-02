#include <iostream>

#include "robot_localization.h"

int main(int argc, char** argv)
{
    int result = EXIT_SUCCESS;
    try
    {
        robot_localization(argc, argv);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        result = EXIT_FAILURE;  // abnormal termination of program
    }

    return result;
}
