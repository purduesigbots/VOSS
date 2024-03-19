#include "VOSS/exit_conditions/ToleranceAngularExitCondition.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>

namespace voss::controller {

ToleranceAngularExitCondition::ToleranceAngularExitCondition(double tolerance)
    : tolerance(tolerance) {
}

bool ToleranceAngularExitCondition::is_met(Pose current_pose, bool thru) {
    // if there is no target angle, return false so the other conditions run.
    if (!this->target_pose.theta.has_value()) {
        return false;
    }
    return std::abs(voss::norm_delta(current_pose.theta.value() -
                                     this->target_pose.theta.value())) <
           this->tolerance;
}

} // namespace voss::controller