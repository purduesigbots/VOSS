#include "TimeOutExitCondition.hpp"

namespace voss::controller {
    bool TimeOutExitCondition::is_met(Pose current_pose) {
        return current_time >= timeout;
    }

    void TimeOutExitCondition::set_target(Pose target) {
        target_pose = target;
    }

    void TimeOutExitCondition::set_timeout(int timeout) {
        this->timeout = timeout;
    }
}