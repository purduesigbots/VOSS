
#include "VOSS/exit_conditions/ToleranceExitCondition.hpp"

namespace voss::controller {

bool ToleranceExitCondition::is_met(Pose pose, bool thru) {
    if (this->target_has_coordinate() && this->target_has_heading()) {
        return ang_exit->is_met(pose, thru) && lin_exit->is_met(pose, thru);
    } else if (this->target_has_coordinate()) {
        return lin_exit->is_met(pose, thru);
    } else if (this->target_has_heading()) {
        return ang_exit->is_met(pose, thru);
    }
    return false;
}

void ToleranceExitCondition::set_target(Pose target) {
    AbstractExitCondition::set_target(target);
    if (this->ang_exit) {
        this->ang_exit->set_target(target);
    }
    if (this->lin_exit) {
        this->lin_exit->set_target(target);
    }
}

void ToleranceExitCondition::reset() {
    if (this->ang_exit) {
        this->ang_exit->reset();
    }
    if (this->lin_exit) {
        this->lin_exit->reset();
    }
}

void ToleranceExitCondition::add_ang_exit(double angular_tolerance,
                                          double tolerance_time) {
    this->ang_exit = std::make_shared<ToleranceAngularExitCondition>(
        angular_tolerance, tolerance_time);
}

void ToleranceExitCondition::add_lin_exit(double linear_tolerance,
                                          double tolerance_time) {
    this->lin_exit = std::make_shared<ToleranceLinearExitCondition>(
        linear_tolerance, tolerance_time);
}
} // namespace voss::controller
