#pragma once
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace MyHelpers{

class HelperTime
{

public:


    static std::string GetCurrentTimeStr(){

        std::string timeStr = "";
        std::time_t now = std::time(nullptr);
        std::tm* localTime = std::localtime(&now);
        char buffer[100];
        if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", localTime)) {
            timeStr = buffer;
        }
        return timeStr;
    }
};
};