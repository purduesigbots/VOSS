#include "VOSS/exit_conditions/TimeOutExitCondition.hpp"

namespace voss::controller {

TimeOutExitCondition::TimeOutExitCondition(int timeout) : timeout(timeout) {
    this->current_time = 0;
}

bool TimeOutExitCondition::is_met(Pose current_pose) {
    this->current_time += 10;
    if (current_time >= this->timeout) {
        this->current_time = 0;
        return true;
    } else {
        return false;
    }
}

} // namespace voss::controller