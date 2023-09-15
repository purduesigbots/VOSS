#include "legs/chassis/DiffChassis.hpp"
#include "pros/motors.h"

namespace legs::chassis {

DiffChassis::DiffChassis(std::initializer_list<int8_t> left_motors,
                         std::initializer_list<int8_t> right_motors,
                         controller::AbstractController& default_controller)
    : AbstractChassis(left_motors, right_motors, default_controller) {
}

void DiffChassis::tank(double left_speed, double right_speed) {
	this->left_motors->move_voltage(120.0 * left_speed);
	this->right_motors->move_voltage(120.0 * right_speed);
}

void DiffChassis::arcade(double forward_speed, double turn_speed) {
	double left = forward_speed + turn_speed;
	double right = forward_speed - turn_speed;

	this->left_motors->move_voltage(120.0 * left);
	this->right_motors->move_voltage(120.0 * right);
}

bool DiffChassis::execute(ChassisCommand cmd) {
	return std::visit(
	    overload{[=](Stop&) -> bool {
		             this->left_motors->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
		             this->right_motors->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
		             this->left_motors->move_voltage(0);
		             this->right_motors->move_voltage(0);

		             return true;
	             },
	             [=](Voltages& v) -> bool {
		             this->left_motors->move_voltage(120 * v.left);
		             this->right_motors->move_voltage(120 * v.right);

		             return false;
	             }},
	    cmd);
}

} // namespace legs::chassis