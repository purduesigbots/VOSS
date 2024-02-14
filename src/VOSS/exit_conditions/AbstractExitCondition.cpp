#include "VOSS/exit_conditions/AbstractExitCondition.hpp"

namespace voss::controller {

void AbstractExitCondition::set_target(Pose target) {
    this->target_pose = target;
}

} // namespace voss::controller