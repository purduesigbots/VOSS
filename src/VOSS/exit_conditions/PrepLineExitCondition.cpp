#include "VOSS/exit_conditions/PrepLineExitCondition.hpp"
#include "VOSS/utils/Point.hpp"
#include <cstdio>

namespace voss::controller {
PrepLineExitCondition::PrepLineExitCondition(double thru_smoothness)
    : thru_smoothness(thru_smoothness) {
}

bool PrepLineExitCondition::is_met(voss::Pose pose, bool thru) {
    if (thru) {
        if (this->target_pose.theta.has_value()) { // semi circle exit
            bool exit = (pose.y - this->target_pose.y) *
                                cos(target_pose.theta.value() - M_PI_2) <
                            (pose.x - target_pose.x) *
                                sin(target_pose.theta.value() - M_PI_2) &&
                        (pow((pose.y - target_pose.y), 2) +
                         pow((pose.x - target_pose.x), 2)) <
                            pow(this->thru_smoothness * 2, 2);
            // if (exit) {
            //     printf("Prep line cond met 1\n");
            // }
            return exit;
        } else { // line exit
                 //
                 //             /*thru &&
                 //                 current_pos.y + this->min_error >=
            //                     (-1.0 / m) * (current_pos.x + min_error -
            //                     virtualTarget.x) +
            //                 virtualTarget.y*/
            //            double m = fabs((pose.y - this->target_pose.y)
            //                            / (pose.x - this->target_pose.x));
            //            bool exit = pose.y + this->thru_smoothness >=
            //                (-1.0 / m) * (pose.x + thru_smoothness -
            //                target_pose.x) +
            //                    target_pose.y;
            double d = voss::Point::getDistance(
                {this->target_pose.x, this->target_pose.y}, {pose.x, pose.y});
            bool exit = d < this->thru_smoothness;
            // if (exit) {
            //     printf("Prep line cond met 2\n");
            // }
            return exit;
        }
    }
    return false;
}
}; // namespace voss::controller
