
#include "VOSS/exit_conditions/ToleranceExitCondition.hpp"

namespace voss::controller {

bool ToleranceExitCondition::is_met(Pose pose, bool thru) {
    if (this->target_has_coordinate() && this->target_has_heading()) {
        return ang_exit->is_met(pose, thru) && ang_exit->is_met(pose, thru);
    } else if (this->target_has_coordinate()) {
        return lin_exit->is_met(pose, thru);
    } else if (this->target_has_heading()) {
        return ang_exit->is_met(pose, thru);
    }
    return false;
}
void ToleranceExitCondition::add_ang_exit(double angular_tolerance) {
    this->ang_exit =
        std::make_shared<ToleranceAngularExitCondition>(angular_tolerance);
}

void ToleranceExitCondition::add_lin_exit(double linear_tolerance) {
    this->lin_exit =
        std::make_shared<ToleranceLinearExitCondition>(linear_tolerance);
}
} // namespace voss::controller
