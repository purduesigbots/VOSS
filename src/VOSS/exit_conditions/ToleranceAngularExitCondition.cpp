#include "VOSS/exit_conditions/ToleranceAngularExitCondition.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>
#include <cstdio>

namespace voss::controller {

ToleranceAngularExitCondition::ToleranceAngularExitCondition(double tolerance, double tolerance_time)
    : tolerance(voss::to_radians(tolerance)), tolerance_time(tolerance_time) {
}

bool ToleranceAngularExitCondition::is_met(Pose current_pose, bool thru) {
    if (std::abs(voss::norm_delta(current_pose.theta.value() -
                                     this->target_pose.theta.value())) <
           this->tolerance) {
        if (thru) {
            return true;
        }
        current_time += 10;
    } else {
        current_time = 0;
    }
    return current_time > tolerance_time;
}

void ToleranceAngularExitCondition::reset() {
    current_time = 0;
}

} // namespace voss::controller