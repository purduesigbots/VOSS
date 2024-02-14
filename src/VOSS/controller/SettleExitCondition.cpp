#include "VOSS/controller/SettleExitCondition.hpp"
#include <cmath>

namespace voss::controller {
    bool SettleExitCondition::is_met(Pose current_pose) {
        if (current_time < settle_time) {
            if (std::abs(current_pose.x - prev_pose.x) < tolerance && std::abs(current_pose.y - prev_pose.y) < tolerance && std::abs(current_pose.theta - prev_pose.theta) < tolerance){
                current_time++;
            } else {
                current_time = 0;
                prev_pose = current_pose;
            }
        } else {
            return true;
        }
        return false;
    }

    void SettleExitCondition::set_settle_time(int settle_time) {
        this->settle_time = settle_time;
    }

    void SettleExitCondition::set_tolerance(double tolerance) {
        this->tolerance = tolerance;
    }
} // namespace voss::controller