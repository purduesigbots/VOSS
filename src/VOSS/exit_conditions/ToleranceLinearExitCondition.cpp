#include "VOSS/exit_conditions/ToleranceLinearExitCondition.hpp"
#include "VOSS/utils/Point.hpp"
#include <cmath>
#include <cstdio>

namespace voss::controller {

ToleranceLinearExitCondition::ToleranceLinearExitCondition(
    double tolerance, double tolerance_time)
    : tolerance(tolerance), tolerance_time(tolerance_time) {
}

bool ToleranceLinearExitCondition::is_met(Pose current_pose, bool thru) {
    //    printf("current pose: %f %f\n", current_pose.x, current_pose.y);
    //    printf("target pose: %f %f\n", target_pose.x, target_pose.y);
    double d =
        voss::Point::getDistance({this->target_pose.x, this->target_pose.y},
                                 {current_pose.x, current_pose.y});
    if (d < this->tolerance) {
        if (thru) {
            return true;
        }
        current_time += 10;
    } else {
        current_time = 0;
    }
    return current_time > tolerance_time;
}

void ToleranceLinearExitCondition::reset() {
    current_time = 0;
}

} // namespace voss::controller