#include "XDriveChassis.hpp"
#include <cmath>

voss::chassis::XDriveChassis::XDriveChassis(
		int8_t front_left_motor, int8_t back_left_motor, int8_t front_right_motor,
		int8_t back_right_motor,
		controller::AbstractController &default_controller) : AbstractChassis(default_controller) {
	this->front_left_motor = std::make_unique<pros::Motor>(front_left_motor);
	this->back_left_motor = std::make_unique<pros::Motor>(back_left_motor);
	this->front_right_motor = std::make_unique<pros::Motor>(front_right_motor);
	this->back_right_motor = std::make_unique<pros::Motor>(back_right_motor);
}

void voss::chassis::XDriveChassis::holonomic(double forward_speed, double turn_speed, double strafe_speed) {
	this->front_left_motor->move_voltage(120 * (forward_speed + turn_speed + strafe_speed));
	this->back_left_motor->move_voltage(120 * (forward_speed + turn_speed - strafe_speed));
	this->front_right_motor->move_voltage(120 * (forward_speed - turn_speed - strafe_speed));
	this->back_right_motor->move_voltage(120 * (forward_speed - turn_speed + strafe_speed));
}

void voss::chassis::XDriveChassis::tank(double left_speed, double right_speed) {
	this->front_left_motor->move_voltage(120 * left_speed);
	this->back_left_motor->move_voltage(120 * left_speed);
	this->front_right_motor->move_voltage(120 * right_speed);
	this->back_right_motor->move_voltage(120 * right_speed);
}

void voss::chassis::XDriveChassis::arcade(double forward_speed, double turn_speed) {
	this->front_left_motor->move_voltage(120 * (forward_speed + turn_speed));
	this->back_left_motor->move_voltage(120 * (forward_speed + turn_speed));
	this->front_right_motor->move_voltage(120 * (forward_speed - turn_speed));
	this->back_right_motor->move_voltage(120 * (forward_speed - turn_speed));
}

bool voss::chassis::XDriveChassis::execute(ChassisCommand cmd, double max) {
	return std::visit(overload{[this](Stop&) -> bool {
			this->front_left_motor->move_voltage(0);
			this->back_left_motor->move_voltage(0);
			this->front_right_motor->move_voltage(0);
			this->back_right_motor->move_voltage(0);

			return true;
		},
		[this, max](Voltages& v) -> bool {
			double front_left = v.linear + v.angular;
			double back_left = v.linear + v.angular;
			double front_right = v.linear - v.angular;
			double back_right = v.linear - v.angular;

			if (fabs(front_left) > max)
				front_left = max * (front_left > 0 ? 1 : -1);
			if (fabs(back_left) > max)
				back_left = max * (back_left > 0 ? 1 : -1);
			if (fabs(front_right) > max)
				front_right = max * (front_right > 0 ? 1 : -1);
			if (fabs(back_right) > max)
				back_right = max * (back_right > 0 ? 1 : -1);

			this->front_left_motor->move_voltage(120 * front_left);
			this->back_left_motor->move_voltage(120 *  back_left);
			this->front_right_motor->move_voltage(120 * front_right);
			this->back_right_motor->move_voltage(120 * back_right);

			return false;
		}
	}, cmd);
}
