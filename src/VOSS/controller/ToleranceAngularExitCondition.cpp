#include "VOSS/controller/ToleranceAngularExitCondition.hpp"
#include <cmath>

namespace voss::controller {
    bool ToleranceAngularExitCondition::is_met(Pose current_pose) {
        if (std::abs(current_pose.theta - target_pose.theta) < tolerance) {
            return true;
        }
        return false;
    }

    void ToleranceAngularExitCondition::set_target(Pose target) {
        target_pose = target;
    }

    void ToleranceAngularExitCondition::set_tolerance(double tolerance) {
        this->tolerance = tolerance;
    }
} // namespace voss::controller