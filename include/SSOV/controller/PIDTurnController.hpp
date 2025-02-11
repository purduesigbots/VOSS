#pragma once

#include "SSOV/controller/PIDContoller.hpp"
#include "SSOV/controller/TurnController.hpp"

namespace ssov {

class PIDTurnController: public TurnController {
    private:
        PIDController turn_pid;
        const double min_error;
    public:
        PIDTurnController(PIDConstants turn_constants, double min_error):
            turn_pid(turn_constants),
            min_error(min_error) {};

        double compute(double current_heading, double target_heading, TurnDirection direction, bool thru) override;
        void reset() override;
};

}