#include "legs/controller/PIDController.hpp"
#include "legs/chassis/ChassisCommand.hpp"
#include <cmath>

namespace legs::controller {

PIDController::PIDController(localizer::AbstractLocalizer& l)
    : AbstractController(l), prev_lin_err(0.0), total_lin_err(0.0),
      prev_ang_err(0.0), total_ang_err(0.0) {
}

chassis::ChassisCommand PIDController::get_command(Pose target) {
	Point current_pos = this->l->get_position();

	double dx = target.x - current_pos.x;
	double dy = target.y - current_pos.y;

	double distance_error = sqrt(dx * dx + dy * dy);
	if (distance_error <= exit_error) {
		return chassis::ChassisCommand{chassis::Stop{}};
	}

	double angle_error = atan2(dy, dx) - this->l->get_orientation_rad();

	while (fabs(angle_error) > M_PI) {
		angle_error -= 2 * M_PI * angle_error / fabs(angle_error);
	}

	double lin_speed = linear_pid(distance_error);
	double ang_speed = angular_pid(angle_error);

	return chassis::ChassisCommand{
	    chassis::Voltages{lin_speed - ang_speed, lin_speed + ang_speed}};
}

double PIDController::linear_pid(double error) {
	total_lin_err += error;

	double speed = linear_kP * error + linear_kD * (error - prev_lin_err) +
	               linear_kI * total_lin_err;

	this->prev_lin_err = error;

	return speed;
}

double PIDController::angular_pid(double error) {
	total_ang_err += error;

	double speed = angular_kP * error + angular_kD * (error - prev_ang_err) +
	               angular_kI * total_ang_err;

	this->prev_ang_err = error;

	return speed;
}

void PIDController::reset() {
	this->prev_lin_err = 0;
	this->total_lin_err = 0;
	this->prev_ang_err = 0;
	this->total_ang_err = 0;
}

} // namespace legs::controller