#include "VOSS/exit_conditions/ToleranceAngularExitCondition.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>
#include <cstdio>

namespace voss::controller {

ToleranceAngularExitCondition::ToleranceAngularExitCondition(double tolerance)
    : tolerance(voss::to_radians(tolerance)) {
}

bool ToleranceAngularExitCondition::is_met(Pose current_pose, bool thru) {
    return std::abs(voss::norm_delta(current_pose.theta.value() -
                                     this->target_pose.theta.value())) <
           this->tolerance;
}

} // namespace voss::controller