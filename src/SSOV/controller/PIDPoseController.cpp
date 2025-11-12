#include "SSOV/controller/PIDPoseController.hpp"

#include "SSOV/common/Math.hpp"

#include <iostream>

#include <algorithm>

#include <numbers>

//#include <math.h>

namespace ssov {
DriveSignal PIDPoseController::compute(const Pose &current_pose, const Pose &target_point, bool reverse, bool thru, bool holonomic, float strafe_angle) {
    int dir = reverse ? -1 : 1;
    double dx = target_point.x - current_pose.x;
    double dy = target_point.y - current_pose.y;

    //printf("DX: %f DY: %f\n", dx, dy);
    double distance_error = sqrt(dx * dx + dy * dy);
    double angle_error;
    double lin_speed;
    double hor_speed;
    if (!reverse) {
        angle_error = atan2(dy, dx) - current_pose.theta;
    } else {
        angle_error = atan2(-dy, -dx) - current_pose.theta;
    }

    //We don't want things to blow up
    if (angle_error == NAN){
        angle_error = 0;
    }

    //If we are on a holonomic drive we have different logic
    if (holonomic){
        //We split our speed into linear and horizontal speeds for our drive train
        double lin_speed = (thru ? 100.0 : (linear_pid.update(cos(distance_error)))) * dir;
        double hor_speed = (thru ? 100.0 : (horizontal_pid.update(sin(distance_error)))) * dir;
    }
    else {
        lin_speed = (thru ? 100.0 : (linear_pid.update(distance_error))) * dir;
    }

    //We want the ability to strafe
    //As long as our strafe angle is within [-2p, 2pi] we actually strafe
    //To not use a strafe angle just set the strafe angle higher than 2pi or lower than -2pi
    if(std::abs(strafe_angle) <= 2 * M_PI){
        angle_error = strafe_angle - current_pose.theta;
    }

    //Norm delta the angle
    angle_error = norm_delta(angle_error);
    
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
        double min_dist_ang_err = norm_delta(target_point.theta - current_pose.theta);
        ang_speed = angular_pid.update(min_dist_ang_err);
        //printf("Angular speed norm: %0.2f, Angle: %0.2f, Desired Angle: %0.2f\n", ang_speed, current_pose.theta, target_point.theta);
    } else {
        //FIX THIS OR YOU DIE YOU GOOBERS IT WILL NOT LOSE MORE SLEEP
        //AHGHGHGHGHGHGHHGHHHHHHHHHHH
        //If we are close enough to our target we want to turn to our target pose
        if (distance_error < min_error * final_angle_multiplier)
            angle_error = norm_delta(target_point.theta - current_pose.theta);
            
        min_dist_angle = NAN;
        if (fabs(angle_error) > M_PI && this->can_reverse) {
            angle_error = angle_error - (std::signbit(angle_error) ? -1 : 1) * M_PI;
            lin_speed = -lin_speed;
        }
        ang_speed = angular_pid.update(angle_error);
        //printf("  Angular speed: %f\n", ang_speed);
    }

    //Clamp our stuff
    lin_speed = std::clamp(lin_speed, -100.0, 100.0);
    hor_speed = std::clamp(hor_speed, -100.0, 100.0);
    if (debug) {
        printf("%.2f, %.2f\n", distance_error, angle_error);
    }
    //printf("Angle speed: %f  Lin speed: %f  Hor speed: %f  Angle error: %f DX: %f DY: %f\n", ang_speed, lin_speed, hor_speed, angle_error, dx, dy);
    return {lin_speed, hor_speed, ang_speed};
}

void PIDPoseController::reset() {
    linear_pid.reset();
    angular_pid.reset();
    can_reverse = false;
    min_dist_angle = NAN;
}
}