#include "SSOV/controller/PIDPointController.hpp"

#include "SSOV/common/Angle.hpp"

#include <cmath>

namespace ssov {
ChassisSpeeds PIDPointController::compute(const Pose &current_pose, const Point &target_point, const bool &reverse, const bool &thru) {
    int dir = reverse ? -1 : 1;
    double dx = target_point.x - current_pose.x;
    double dy = target_point.y - current_pose.y;
    double distance_error = sqrt(dx * dx + dy * dy);
    double angle_error = atan2(dy, dx) - current_pose.theta;
    if (reverse) {
        angle_error = atan2(-dy, -dx) - current_pose.theta;
    }
    angle_error = norm_delta(angle_error);
    double lin_speed =
        (thru ? 100.0 : (linear_pid.update(distance_error))) * dir;

    double ang_speed;
    if (distance_error < min_error) {
        this->can_reverse = true;

        ang_speed = 0; // disable turning when close to the point to prevent
                           // spinning

        // reduce the linear speed if the bot is tangent to the target
        lin_speed *= cos(angle_error);

    } else if (distance_error < 2 * min_error) {
        // scale angular speed down to 0 as distance_error approaches min_error
        ang_speed = angular_pid.update(angle_error);
        ang_speed *= (distance_error - min_error) / min_error;
    } else {
        if (fabs(angle_error) > M_PI_2 && this->can_reverse) {
            angle_error =
                angle_error - (std::signbit(angle_error) ? -1 : 1) * M_PI;
            lin_speed = -lin_speed;
        }

        ang_speed = angular_pid.update(angle_error);
    }

    lin_speed = std::clamp(lin_speed, -100.0, 100.0);
    return {lin_speed - ang_speed, lin_speed + ang_speed};
}

void PIDPointController::reset() {
    linear_pid.reset();
    angular_pid.reset();
    can_reverse = false;
}
}