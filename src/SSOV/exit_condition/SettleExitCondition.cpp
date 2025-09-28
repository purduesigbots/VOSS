#include "SSOV/exit_condition/SettleExitCondition.hpp"

#include "pros/rtos.hpp"
#include "SSOV/common/Math.hpp"

namespace ssov {

bool SettleExitCondition::is_met(const Pose &current_pose, const Pose &target_pose, bool thru) {
    uint32_t current_time = pros::millis();
    if (current_time - initial_time <= initial_delay) {
        return false;
    }
    if (distance(current_pose.to_point(), prev_pose.to_point()) < linear_tolerance
        && fabs(norm_delta(current_pose.theta - prev_pose.theta)) < angular_tolerance) {
        settled_time += current_time - prev_time;
        if (settled_time >= settle_time) {
            return true;
        }
    } else {
        settled_time = 0;
    }
    prev_time = current_time;
    prev_pose = current_pose;
    return false;
}

void SettleExitCondition::reset() {
    initial_time = pros::millis();
    settled_time = 0;
    prev_time = pros::millis();
    prev_pose = {};
}

}