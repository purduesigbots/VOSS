#pragma once

#include "AbstractChassis.hpp"
#include "ChassisCommand.hpp"
#include "pros/motors.hpp"
#include <memory>

namespace voss::chassis {

class XDriveChassis : public AbstractChassis {

private:
	std::unique_ptr<pros::Motor> front_left_motor;
	std::unique_ptr<pros::Motor> back_left_motor;
	std::unique_ptr<pros::Motor> front_right_motor;
	std::unique_ptr<pros::Motor> back_right_motor;

public:
	XDriveChassis(int8_t front_left_motor,
								int8_t back_left_motor,
								int8_t front_right_motor,
								int8_t back_right_motor,
								controller::AbstractController& default_controller);

	void holonomic(double forward_speed, double turn_speed, double strafe_speed);
	void tank(double left_speed, double right_speed);
	void arcade(double forward_speed, double turn_speed);

	bool execute(ChassisCommand cmd, double max);
};

}