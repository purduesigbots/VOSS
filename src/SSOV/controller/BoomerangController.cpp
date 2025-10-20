#include "SSOV/controller/BoomerangController.hpp"

#include <algorithm>

namespace ssov {

DriveSignal BoomerangController::compute(const Pose &current_pose, const Pose &target_pose, bool reverse, bool thru, bool holonomic, float strafe_angle) {
    int dir = reverse ? -1 : 1;
    double distance_error = distance(target_pose.to_point(), current_pose.to_point());
    Point carrot_point;
    if (!reverse) {
        carrot_point = (target_pose - Point{distance_error * lead_pct, 0}).to_point();
    } else {
        carrot_point = (target_pose + Point{distance_error * lead_pct, 0}).to_point();
    }
    double dx = carrot_point.x - current_pose.x;
    double dy = carrot_point.y - current_pose.y;
    double angle_error;
    if (!reverse) {
        angle_error = norm_delta(atan2(dy, dx) - current_pose.theta);
    } else {
        angle_error = norm_delta(atan2(-dy, -dx) - current_pose.theta);
    }
    if (debug) {
        printf("%f, %f\n", distance_error, angle_error);
    }

    double lin_speed =
        (thru ? 100.0 : (linear_pid.update(distance_error))) * dir;

    double ang_speed;
    // similar to PIDPointController, we set the angular PID target to be the
    // target pose heading, but we scale it between min_error and min_error / 2
    double final_target_ang_err = norm_delta(target_pose.theta - current_pose.theta);
    if (distance_error < min_error / 2) {
        this->can_reverse = true;

        ang_speed = angular_pid.update(final_target_ang_err);

        // reduce the linear speed if the bot is tangent to the target
        lin_speed *= cos(angle_error);

    } else if (distance_error < min_error) {
        // scale angular speed down to 0 as distance_error approaches min_error
        double scale_factor = (distance_error - min_error / 2) / min_error * 2;
        double scaled_angle_error = norm_delta(scale_factor * angle_error + (1 - scale_factor) * final_target_ang_err);
        ang_speed = angular_pid.update(scaled_angle_error);
        lin_speed *= cos(scaled_angle_error);
    } else {
        if (fabs(angle_error) > M_PI_2 && this->can_reverse) {
            angle_error =
                angle_error - (std::signbit(angle_error) ? -1 : 1) * M_PI;
            lin_speed = -lin_speed;
        }

        ang_speed = angular_pid.update(angle_error);
    }

    // prioritize turning when speeds are above 100.
    if (fabs(lin_speed) + fabs(ang_speed) > 100) {
        ang_speed = std::clamp(ang_speed, -100.0, 100.0);
        lin_speed = std::clamp(lin_speed, -100 + fabs(ang_speed), 100 - fabs(ang_speed));
    }

    lin_speed = std::clamp(lin_speed, -100.0, 100.0);
    return {lin_speed, 0, ang_speed};
}

void BoomerangController::reset() {
    linear_pid.reset();
    angular_pid.reset();
    can_reverse = false;
}

}