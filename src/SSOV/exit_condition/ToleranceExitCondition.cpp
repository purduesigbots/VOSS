#include "SSOV/exit_condition/ToleranceExitCondition.hpp"

#include "pros/rtos.hpp"
#include "SSOV/common/Math.hpp"

namespace ssov {

bool ToleranceExitCondition::is_met(const Pose &current_pose, const Pose &target_pose, bool thru) {
    double lin_error = distance(target_pose.to_point(), current_pose.to_point());
    double ang_error = norm_delta(target_pose.theta - current_pose.theta);
    if (lin_error < linear_tolerance && ang_error < angular_tolerance) {
        if (thru) return true;
        uint32_t dt = pros::c::millis() - prev_time;
        in_tolerance_time += dt;
    } else {
        in_tolerance_time = 0;
    }
    prev_time = pros::c::millis();
    return in_tolerance_time >= tolerance_time;
}

void ToleranceExitCondition::reset() {
    in_tolerance_time = 0;
    prev_time = pros::c::millis();
}

}