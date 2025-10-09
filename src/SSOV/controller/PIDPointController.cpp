#include "SSOV/controller/PIDPointController.hpp"

#include "SSOV/common/Math.hpp"

#include <iostream>

#include <algorithm>

namespace ssov {
DriveSignal PIDPointController::compute(const Pose &current_pose, const Point &target_point, bool reverse, bool thru, bool holonomic) {
    int dir = reverse ? -1 : 1;
    double dx = target_point.x - current_pose.x;
    double dy = target_point.y - current_pose.y;
    double distance_error = sqrt(dx * dx + dy * dy);
    double angle_error;
    double lin_speed;
    double hor_speed;
    if (!reverse) {
        angle_error = atan2(dy, dx) - current_pose.theta;
    } else {
        angle_error = atan2(-dy, -dx) - current_pose.theta;
    }
    angle_error = norm_delta(angle_error);

    if (angle_error == NAN){
        angle_error = 0;
    }
    
    if (holonomic){
        double direct_speed = (thru ? 100.0 : (linear_pid.update(distance_error))) * dir;
        lin_speed = direct_speed * cos(angle_error);
        hor_speed = direct_speed * sin(angle_error);
        //printf("Direct speed: %f   Lin speed: %f  Hor speed: %f\n", direct_speed, lin_speed, hor_speed);
    }
    else {
        lin_speed = (thru ? 100.0 : (linear_pid.update(distance_error))) * dir;
    }
    
    double ang_speed;
    if (distance_error < min_error) {
        this->can_reverse = true;

        if (!holonomic){
        // reduce the linear speed if the bot is tangent to the target
        lin_speed *= cos(angle_error);
        hor_speed = 0;
        }
        if (std::isnan(min_dist_angle)) {
            min_dist_angle = current_pose.theta;
        }
        double min_dist_ang_err = norm_delta(min_dist_angle - current_pose.theta);
        ang_speed = angular_pid.update(min_dist_ang_err);
    } else {
        min_dist_angle = NAN;
        if (fabs(angle_error) > M_PI_2 && this->can_reverse) {
            angle_error =
                angle_error - (std::signbit(angle_error) ? -1 : 1) * M_PI;
            lin_speed = -lin_speed;
        }

        ang_speed = angular_pid.update(angle_error);
    }

    lin_speed = std::clamp(lin_speed, -100.0, 100.0);
    hor_speed = std::clamp(lin_speed, -100.0, 100.0);
    if (debug) {
        printf("%.2f, %.2f\n", distance_error, angle_error);
    }
    return {lin_speed, hor_speed, ang_speed};
}

void PIDPointController::reset() {
    linear_pid.reset();
    angular_pid.reset();
    can_reverse = false;
    min_dist_angle = NAN;
}
}