#include "VOSS/exit_conditions/ToleranceLinearExitCondition.hpp"
#include "VOSS/utils/Point.hpp"
#include <cmath>
#include <cstdio>

namespace voss::controller {

ToleranceLinearExitCondition::ToleranceLinearExitCondition(double tolerance)
    : tolerance(tolerance) {
}

bool ToleranceLinearExitCondition::is_met(Pose current_pose, bool thru) {
    //    printf("current pose: %f %f\n", current_pose.x, current_pose.y);
    //    printf("target pose: %f %f\n", target_pose.x, target_pose.y);
    double d =
        voss::Point::getDistance({this->target_pose.x, this->target_pose.y},
                                 {current_pose.x, current_pose.y});
    bool met = d < this->tolerance;
    return met;
}

} // namespace voss::controller