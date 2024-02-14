#include "VOSS/controller/ToleranceLinearExitCondition.hpp"
#include <cmath>

namespace voss::controller {
    bool ToleranceLinearExitCondition::is_met(Pose current_pose) {
        if (std::abs(current_pose.x - target_pose.x) < tolerance && std::abs(current_pose.y -target_pose.y) < tolerance) {
            return true;
        } 
        return false;
    }

    void ToleranceLinearExitCondition::set_target(Pose target) {
        target_pose = target;
    }

    void ToleranceLinearExitCondition::set_tolerance(double tolerance) {
        this->tolerance = tolerance;
    }
} // namespace voss::controller