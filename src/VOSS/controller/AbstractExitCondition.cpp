#include "VOSS/controller/AbstractExitCondition.hpp"

namespace voss::controller {
    void AbstractExitCondition::set_target(Pose target) {
        target_pose = target;
    }
}