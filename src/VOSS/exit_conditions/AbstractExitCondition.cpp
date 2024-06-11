#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "iostream"

namespace voss::controller {

void AbstractExitCondition::set_target(Pose target) {
    this->target_pose = target;
    printf("im here12\n");
}

void AbstractExitCondition::reset() {
}
bool AbstractExitCondition::target_has_coordinate() const {
    return this->target_pose.x == this->target_pose.x &&
           this->target_pose.y == this->target_pose.y;
}
bool AbstractExitCondition::target_has_heading() const {
    return this->target_pose.theta.has_value();
}

} // namespace voss::controller