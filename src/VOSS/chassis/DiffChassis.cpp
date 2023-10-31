#include "voss/chassis/DiffChassis.hpp"
#include "pros/motors.h"
#include <cmath>

namespace voss::chassis {

double DiffChassis::slew(double target, bool is_left) {
	double step = this->slew_step;
	double current =
	    is_left ? (this->prev_voltages.linear - this->prev_voltages.angular) : (this->prev_voltages.linear + this->prev_voltages.angular);

	if (fabs(current) > fabs(target))
		step = 200;

	if (target > current + step)
		current += step;
	else if (target < current - step)
		current -= step;
	else
		current = target;

	return current;
}

DiffChassis::DiffChassis(std::initializer_list<int8_t> left_motors,
                         std::initializer_list<int8_t> right_motors,
                         controller::AbstractController& default_controller,
                         double slew_step)
    : AbstractChassis(default_controller) {
	this->left_motors = std::make_unique<pros::MotorGroup>(left_motors);
	this->right_motors = std::make_unique<pros::MotorGroup>(right_motors);

	this->slew_step = slew_step > 0 ? slew_step : 200;
	this->prev_voltages = {0, 0};
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
			double left = v.linear - v.angular;
			double right = v.linear + v.angular;

			if (fabs(left) > max) {
				left = max * ((left < 0) ? -1 : 1);
			}
			if (fabs(right) > max) {
				right = max * ((right < 0) ? -1 : 1);
			}

			left = slew(left, true);
			right = slew(right, false);

			this->left_motors->move_voltage(120 * left);
			this->right_motors->move_voltage(120 * right);

			this->prev_voltages = v;

			return false;
		}},
		cmd
	);
}

} // namespace voss::chassis