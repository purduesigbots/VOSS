#include "VOSS/exit_conditions/SettleExitCondition.hpp"
#include "VOSS/utils/Pose.hpp"
#include <cmath>

namespace voss::controller {
bool SettleExitCondition::is_met(Pose current_pose) {

    if (this->current_time < this->settle_time) {
        if (std::abs(current_pose.x - this->prev_pose.x) < this->tolerance &&
            std::abs(current_pose.y - this->prev_pose.y) < this->tolerance &&
            std::abs(current_pose.theta - this->prev_pose.theta) <
                this->tolerance) {
            this->current_time += 10;
        } else {
            current_time = 0;
            prev_pose = current_pose;
        }
    } else {
        this->current_time = 0;
        this->prev_pose = Pose{0, 0, 0};
        return true;
    }
    return false;
}

SettleExitCondition::SettleExitCondition(Pose target_pose, int settle_time,
                                         double tolerance)
    : settle_time(settle_time), tolerance(tolerance) {
    this->target_pose = target_pose;
    this->current_time = 0;
    this->prev_pose = Pose{0, 0, 0};
}

} // namespace voss::controller