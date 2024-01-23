#pragma once

#include "pros/misc.hpp"
#include "pros/motors.hpp"

#include "ChassisCommand.hpp"
#include "voss/controller/AbstractController.hpp"

#include "voss/utils/Point.hpp"
#include "voss/utils/Pose.hpp"
#include "voss/utils/flags.hpp"

namespace voss::chassis {

class AbstractChassis {

protected:
	pros::Mutex m;
	controller::AbstractController* default_controller;

	void move_task(controller::AbstractController* controller, double max,
	               voss::Flags flags);

	void turn_task(controller::AbstractController* controller, double max,
	               voss::Flags flags);

public:
	AbstractChassis(controller::AbstractController& default_controller);

	virtual void tank(double left_speed, double right_speed) = 0;
	virtual void arcade(double forward_speed, double turn_speed) = 0;

	virtual bool execute(ChassisCommand cmd, double max) = 0;

	void move(Point target, controller::AbstractController* controller,
	          double max = 100.0, voss::Flags flags = voss::Flags::NONE);
	void move(Pose target, controller::AbstractController* controller,
	          double max = 100.0, voss::Flags flags = voss::Flags::NONE);
	void move(Point target, double max = 100.0, voss::Flags flags = voss::Flags::NONE);
	void move(Pose target, double max = 100.0, voss::Flags flags = voss::Flags::NONE);

	void turn(double target, controller::AbstractController* controller,
	          double max = 100.0, voss::Flags flags = voss::Flags::NONE);
	void turn(double target, double max = 100.0, voss::Flags flags = voss::Flags::NONE);
	void turn_to(Point target, controller::AbstractController* controller,
	             double max = 100.0, voss::Flags flags = voss::Flags::NONE);
	void turn_to(Point target, double max = 100.0, voss::Flags flags = voss::Flags::NONE);
};

} // namespace voss::chassis