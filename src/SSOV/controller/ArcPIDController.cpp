#include "SSOV/controller/ArcPIDController.hpp"

#include "SSOV/common/Math.hpp"

#include <iostream>

#include <algorithm>

#include <numbers>

//#include <math.h>

namespace ssov {
DriveSignal ArcPIDController::compute(const Pose &current_pose, const Pose &target_point, bool reverse, bool thru, bool holonomic, float strafe_angle) {
    int dir = reverse ? -1 : 1;
    double angle_error;
    double lin_speed;
    double ang_speed;
    double dx = target_point.x - current_pose.x;
    double dy = target_point.y - current_pose.y;
    double distance = sqrt(dx * dx + dy * dy);

    if (!reverse) {
        angle_error = atan2(dy, dx) - current_pose.theta;
    } else {
        angle_error = atan2(-dy, -dx) - current_pose.theta;
    }
    angle_error += distance / 30 * target_point.theta;
    angle_error = norm_delta(angle_error);

    lin_speed = (thru ? 100.0 : (linear_pid.update(distance))) * dir;
    ang_speed = angular_pid.update(angle_error);

    if (fabs(lin_speed) + fabs(ang_speed) > 100) {
        ang_speed = std::clamp(ang_speed, -100.0, 100.0);
        lin_speed = std::clamp(lin_speed, -100 + fabs(ang_speed), 100 - fabs(ang_speed));
    }

    return {lin_speed, 0, ang_speed};

  } 

void ArcPIDController::reset() {
    linear_pid.reset();
    angular_pid.reset();
    can_reverse = false;
    min_dist_angle = NAN;
}
}