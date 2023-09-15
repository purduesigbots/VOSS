#pragma once

#include "pros/misc.hpp"
#include "pros/motors.hpp"

#include "ChassisCommand.hpp"
#include "legs/controller/AbstractController.hpp"
#include "legs/utils/Point.hpp"
#include "legs/utils/Pose.hpp"
#include <initializer_list>
#include <memory>

namespace legs::chassis {

class AbstractChassis {

protected:
	std::unique_ptr<pros::Motor_Group> left_motors;
	std::unique_ptr<pros::Motor_Group> right_motors;
	controller::AbstractController* default_controller;

public:
	AbstractChassis(std::initializer_list<int8_t> left_motors,
	                std::initializer_list<int8_t> right_motors,
	                controller::AbstractController& default_controller);

	virtual void tank(double left_speed, double right_speed) = 0;
	virtual void arcade(double forward_speed, double turn_speed) = 0;

	virtual bool execute(ChassisCommand cmd) = 0;

	void move(Point target, controller::AbstractController* controller);
	void move(Pose target, controller::AbstractController* controller);
	void move(Point target);
	void move(Pose target);
};

} // namespace legs::chassis