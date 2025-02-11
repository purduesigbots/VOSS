#include "SSOV/controller/PIDTurnController.hpp"

#include "SSOV/common/Math.hpp"

namespace ssov {

double PIDTurnController::compute(double current_heading, double target_heading, TurnDirection direction, bool thru) {
    double angle_error = norm_delta(target_heading - current_heading);
    if (fabs(angle_error) > min_error) {
        if (direction == TurnDirection::CCW && angle_error < 0) {
            angle_error += 2 * M_PI;
        } else if (direction == TurnDirection::CW && angle_error > 0) {
            angle_error -= 2 * M_PI;
        }
    }
    return turn_pid.update(angle_error);
}

void PIDTurnController::reset() {
    turn_pid.reset();
}

}