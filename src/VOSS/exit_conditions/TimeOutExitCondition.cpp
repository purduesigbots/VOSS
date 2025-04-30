#include "VOSS/exit_conditions/TimeOutExitCondition.hpp"
#include "VOSS/constants.hpp"
#include "VOSS/utils/debug.hpp"
#include <cstdio>

namespace voss::controller {

TimeOutExitCondition::TimeOutExitCondition(int timeout) : timeout(timeout) {
    this->current_time = 0;
}

bool TimeOutExitCondition::is_met(Pose current_pose, bool thru) {
    this->current_time += constants::MOTOR_UPDATE_DELAY;
    bool exit = current_time >= timeout;
    if (get_debug() && exit) {
        printf("Timeout Condition Met\n");
    }
    return this->current_time >= this->timeout;
}

void TimeOutExitCondition::reset() {
    this->current_time = 0;
}

} // namespace voss::controller