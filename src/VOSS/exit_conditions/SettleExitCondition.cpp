#include "VOSS/exit_conditions/SettleExitCondition.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/utils/angle.hpp"
#include "VOSS/utils/Pose.hpp"
#include <cmath>
#include <cstdio>

namespace voss::controller {
bool SettleExitCondition::is_met(Pose current_pose, bool thru) {
    if (initial_time <= initial_delay) {
        initial_time += 10;
        return false;
    }
    if (this->current_time < this->settle_time) {
        if (std::abs(current_pose.x - this->prev_pose.x) < this->tolerance &&
            std::abs(current_pose.y - this->prev_pose.y) < this->tolerance) {
            if (current_pose.theta.has_value()) {
                if (std::abs(current_pose.theta.value() -
                             this->prev_pose.theta.value()) <
                    voss::to_radians(this->tolerance)) {
                    this->current_time += 10;
                }
            }

            //            if(thru) {
            //                printf("settle condition met\n");
            //                return true;
            //            }

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

SettleExitCondition::SettleExitCondition(int settle_time, double tolerance,
                                         int initial_delay)
    : settle_time(settle_time), tolerance(tolerance),
      initial_delay(initial_delay) {
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