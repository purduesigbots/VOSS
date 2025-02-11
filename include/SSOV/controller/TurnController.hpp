#pragma once

#include "SSOV/chassis/ChassisCommand.hpp"

namespace ssov {

enum class TurnDirection {
    AUTO,
    CW,
    CCW
};

class TurnController {
    public:
        virtual double compute(double current_heading, double target_heading, TurnDirection direction, bool thru) = 0;
        virtual void reset() = 0;
};

}