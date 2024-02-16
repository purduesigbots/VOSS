#include "VOSS/exit_conditions/ToleranceAngularExitCondition.hpp"
#include <cmath>

namespace voss::controller {

ToleranceAngularExitCondition::ToleranceAngularExitCondition(Pose target_pose,
                                                             double tolerance)
    : tolerance(tolerance) {
    this->target_pose = target_pose;
}

bool ToleranceAngularExitCondition::is_met(Pose current_pose) {
    return std::abs(current_pose.theta - this->target_pose.theta) <
           this->tolerance;
}

} // namespace voss::controller