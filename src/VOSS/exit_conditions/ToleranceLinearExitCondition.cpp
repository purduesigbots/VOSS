#include "VOSS/exit_conditions/ToleranceLinearExitCondition.hpp"
#include <cmath>
#include <cstdio>

namespace voss::controller {

ToleranceLinearExitCondition::ToleranceLinearExitCondition(Pose target_pose,
                                                           double tolerance)
    : tolerance(tolerance) {
    this->target_pose = target_pose;
}

bool ToleranceLinearExitCondition::is_met(Pose current_pose) {
    printf("current pose: %f %f\n", current_pose.x, current_pose.y);
    printf("target pose: %f %f\n", target_pose.x, target_pose.y);
    bool met = std::abs(current_pose.x - this->target_pose.x) < this->tolerance &&
           std::abs(current_pose.y - this->target_pose.y) < this->tolerance;
    if (met) {
        printf("linear condition met\n");
    }
    return met;
}

} // namespace voss::controller