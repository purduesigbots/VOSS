#include "VOSS/exit_conditions/ToleranceLinearExitCondition.hpp"
#include <cmath>

namespace voss::controller {

ToleranceLinearExitCondition::ToleranceLinearExitCondition(Pose target_pose,
                                                           double tolerance)
    : tolerance(tolerance) {
    this->target_pose = target_pose;
}

bool ToleranceLinearExitCondition::is_met(Pose current_pose) {
    return std::abs(current_pose.x - this->target_pose.x) < this->tolerance &&
           std::abs(current_pose.y - this->target_pose.y) < this->tolerance;
}

} // namespace voss::controller