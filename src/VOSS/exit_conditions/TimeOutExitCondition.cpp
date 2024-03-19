#include "VOSS/exit_conditions/TimeOutExitCondition.hpp"
#include <cstdio>

namespace voss::controller {

TimeOutExitCondition::TimeOutExitCondition(int timeout) : timeout(timeout) {
    this->current_time = 0;
}

bool TimeOutExitCondition::is_met(Pose current_pose, bool thru) {
    this->current_time += 10;
    // if (this->current_time >= this->timeout)
    //     printf("Time out cond met\n");
    return this->current_time >= this->timeout;
}

void TimeOutExitCondition::reset() {
    this->current_time = 0;
}

} // namespace voss::controller