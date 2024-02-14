#include "VOSS/exit_conditions/TimeOutExitCondition.hpp"

namespace voss::controller {

TimeOutExitCondition::TimeOutExitCondition(int timeout) : timeout(timeout) {
    this->current_time = 0;
}

bool TimeOutExitCondition::is_met(Pose current_pose) {
    return current_time >= this->timeout;
}

} // namespace voss::controller