#include "voss/chassis/DiffChassis.hpp"
#include "pros/motors.h"
#include <cmath>

namespace voss::chassis {

DiffChassis::DiffChassis(std::initializer_list<int8_t> left_motors,
                         std::initializer_list<int8_t> right_motors,
                         controller::AbstractController& default_controller)
    : AbstractChassis(default_controller) {

	this->left_motors = std::make_unique<pros::MotorGroup>(left_motors);
	this->right_motors = std::make_unique<pros::MotorGroup>(right_motors);
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

bool DiffChassis::execute(ChassisCommand cmd, double max) {
	return std::visit(overload{[this](Stop&) -> bool {
		                           this->left_motors->move_voltage(0);
		                           this->right_motors->move_voltage(0);

		                           return true;
	                           },
	                           [this, max](Voltages& v) -> bool {
		                           if (fabs(v.left) > max) {
			                           v.left = max * ((v.left < 0) ? -1 : 1);
		                           }
		                           if (fabs(v.right) > max) {
			                           v.right = max * ((v.right < 0) ? -1 : 1);
		                           }

		                           this->left_motors->move_voltage(120 * v.left);
		                           this->right_motors->move_voltage(120 * v.right);

		                           return false;
	                           }},
	                  cmd);
}

} // namespace voss::chassis