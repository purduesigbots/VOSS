#include "XDriveChassis.hpp"

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
	this->front_left_motor->move_voltage(forward_speed + turn_speed + strafe_speed);
	this->back_left_motor->move_voltage(forward_speed + turn_speed - strafe_speed);
	this->front_right_motor->move_voltage(forward_speed - turn_speed - strafe_speed);
	this->back_right_motor->move_voltage(forward_speed - turn_speed + strafe_speed);
}

void voss::chassis::XDriveChassis::tank(double left_speed, double right_speed) {
	this->front_left_motor->move_voltage(left_speed);
	this->back_left_motor->move_voltage(left_speed);
	this->front_right_motor->move_voltage(right_speed);
	this->back_right_motor->move_voltage(right_speed);
}

void voss::chassis::XDriveChassis::arcade(double forward_speed, double turn_speed) {
	this->front_left_motor->move_voltage(forward_speed + turn_speed);
	this->back_left_motor->move_voltage(forward_speed + turn_speed);
	this->front_right_motor->move_voltage(forward_speed - turn_speed);
	this->back_right_motor->move_voltage(forward_speed - turn_speed);
}

bool voss::chassis::XDriveChassis::execute(ChassisCommand cmd, double max) {
	return std::visit(overload{[this](Stop&) -> bool {
			this->front_left_motor->move_voltage(0);
			this->back_left_motor->move_voltage(0);
			this->front_right_motor->move_voltage(0);
			this->back_right_motor->move_voltage(0);

			return true;
		} // TODO: Add voltages
	}, cmd);
}
