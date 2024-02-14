#include "VOSS/exit_conditions/ToleranceAngularExitCondition.hpp"
#include <cmath>

namespace voss::controller {

ToleranceAngularExitCondition::ToleranceAngularExitCondition(double tolerance)
    : tolerance(tolerance) {
}

bool ToleranceAngularExitCondition::is_met(Pose current_pose) {
    return std::abs(current_pose.theta - this->target_pose.theta) <
           this->tolerance;
}

} // namespace voss::controller