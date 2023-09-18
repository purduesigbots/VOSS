#pragma once

#include "pros/misc.hpp"
#include "pros/motors.hpp"

#include "ChassisCommand.hpp"
#include "legs/controller/AbstractController.hpp"

#include "legs/utils/Point.hpp"
#include "legs/utils/Pose.hpp"
#include "legs/utils/flags.hpp"

namespace legs::chassis {

class AbstractChassis {

protected:
	pros::Mutex m;
	controller::AbstractController* default_controller;

public:
	AbstractChassis(controller::AbstractController& default_controller);

	virtual void tank(double left_speed, double right_speed) = 0;
	virtual void arcade(double forward_speed, double turn_speed) = 0;

	virtual bool execute(ChassisCommand cmd) = 0;

	void move(Point target, controller::AbstractController* controller,
	          uint8_t flags = legs::NONE);
	void move(Pose target, controller::AbstractController* controller,
	          uint8_t flags = legs::NONE);
	void move(Point target, uint8_t flags = legs::NONE);
	void move(Pose target, uint8_t flags = legs::NONE);
};

} // namespace legs::chassis