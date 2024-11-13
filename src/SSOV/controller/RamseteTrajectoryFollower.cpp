#include "SSOV/controller/RamseteTrajectoryFollower.hpp"

namespace ssov {

ChassisSpeeds RamseteTrajectoryFollower::compute(Pose current_pose, Pose current_velocities, TrajectoryState target_state) {
    double x_error = target_state.pose.x - current_pose.x;
    double y_error = target_state.pose.y - current_pose.y;
    double angle_error = target_state.pose.theta - current_pose.theta;
	while (angle_error < -M_PI) {
		angle_error += 2 * M_PI;
	}
	while (angle_error > M_PI) {
		angle_error -= 2 * M_PI;
	}
    double x_error_relative = cos(current_pose.theta) * x_error + sin(current_pose.theta) * y_error;
    double y_error_relative = -sin(current_pose.theta) * x_error + cos(current_pose.theta) * y_error;
	double k = 2 * zeta * sqrt(target_state.vel.theta * target_state.vel.theta + b * target_state.vel.x * target_state.vel.x);
	double vel = target_state.vel.x * cos(angle_error) + k * x_error_relative;
	double sinc = 1;
	if (angle_error != 0) {
		sinc = sin(angle_error) / angle_error;
	}
    double ang_vel = target_state.vel.theta + k * angle_error + b * target_state.vel.x * sinc * y_error_relative;

    double lin_vel_err = vel - current_velocities.x;
    double ang_vel_err = ang_vel - current_velocities.theta;
    double left_speed = kP_lin * lin_vel_err - kP_ang * ang_vel_err + kV_lin * vel - kV_ang * ang_vel;
    double right_speed = kP_lin * lin_vel_err + kP_ang * ang_vel_err + kV_lin * vel + kV_ang * ang_vel;
    return {left_speed, right_speed};
}

}