#include "VOSS/exit_conditions/SettleExitCondition.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/utils/Pose.hpp"
#include <cmath>
#include <cstdio>

namespace voss::controller {
bool SettleExitCondition::is_met(Pose current_pose) {
    if (initial_time <= initial_delay) {
        initial_time += 10;
        return false;
    }
    if (this->current_time < this->settle_time) {
        if (std::abs(current_pose.x - this->prev_pose.x) < this->tolerance &&
            std::abs(current_pose.y - this->prev_pose.y) < this->tolerance &&
            std::abs(current_pose.theta.value() - this->prev_pose.theta.value()) <
                this->tolerance) {
            this->current_time += 10;
        } else {
            current_time = 0;
            prev_pose = current_pose;
        }
    } else {
        printf("settle condition met\n");
        return true;
    }
    return false;
}

SettleExitCondition::SettleExitCondition(Pose target_pose, int settle_time,
                                         double tolerance, int initial_delay)
    : settle_time(settle_time), tolerance(tolerance), initial_delay(initial_delay) {
    this->target_pose = target_pose;
    this->current_time = 0;
    this->initial_time = 0;
    this->prev_pose = Pose{0, 0, 0};
}

void SettleExitCondition::reset() {
    this->current_time = 0;
    this->initial_time = 0;
    this->prev_pose = Pose{0, 0, 0};
}

} // namespace voss::controller