#pragma once

#include <variant>

namespace ssov {
    /*
     * General drive signal compatible with holonomic drives
     * Values range from -100 to 100
     * x - forwards
     * y - sideways
     * theta - angular
     */
    struct DriveSignal {
        double x;
        double y;
        double theta;
    };

    using ChassisCommand = std::variant<DriveSignal>;

    template<class... Ts>
    struct overloaded : Ts... { using Ts::operator()...; };
}