#include "VOSS/exit_conditions/PrepLineExitCondition.hpp"
#include "VOSS/utils/Point.hpp"
#include "VOSS/utils/debug.hpp"
#include <cstdio>

namespace voss::controller {
PrepLineExitCondition::PrepLineExitCondition(double thru_smoothness)
    : thru_smoothness(thru_smoothness) {
}

bool PrepLineExitCondition::is_met(voss::Pose pose, bool thru) {
    if (thru) {
        double d = voss::Point::getDistance(
            {this->target_pose.x, this->target_pose.y}, {pose.x, pose.y});
        bool exit = d < this->thru_smoothness;
        if (get_debug() && exit) {
            printf("Thru Tolerance Condition Met\n");
        }
        return exit;
    }
    return false;
}
}; // namespace voss::controller
