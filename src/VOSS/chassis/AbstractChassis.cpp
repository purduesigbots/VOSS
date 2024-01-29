#include "VOSS/chassis/AbstractChassis.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

namespace voss::chassis {

AbstractChassis::AbstractChassis(
    controller::AbstractController& default_controller) {
	this->default_controller = &default_controller;
}

void AbstractChassis::move_task(controller::AbstractController* controller,
                                double max, voss::Flags flags) {

	pros::Task running_t([&, controller]() {
		controller->reset();
		while (!this->execute(controller->get_command(flags & voss::Flags::REVERSE,
		                                              flags & voss::Flags::THRU),
		                      max)) {
			if (pros::competition::is_disabled()) {
				return;
			}
			pros::delay(10);
		}
		// this->m.give();
	});

	if (flags & voss::Flags::ASYNC) {
		return;
	}

	running_t.join();

	// this->m.take();
	// this->m.give();
}

void AbstractChassis::turn_task(controller::AbstractController* controller,
                                double max, voss::Flags flags) {
	pros::Task running_t([&, controller]() {
		controller->reset();
		while (!this->execute(
		    controller->get_angular_command(flags & voss::Flags::REVERSE,
		                                    flags & voss::Flags::THRU),
		    max)) {
			if (pros::competition::is_disabled()) {
				return;
			}
			pros::delay(10);
		}
	});

	if (flags & voss::Flags::ASYNC) {
		return;
	}

	running_t.join();
}

void AbstractChassis::move(Point target, double max, voss::Flags flags) {
	this->move(target, this->default_controller, max, flags);
}

void AbstractChassis::move(Pose target, double max, voss::Flags flags) {
	this->move(target, this->default_controller, max, flags);
}

void AbstractChassis::move(Point target,
                           controller::AbstractController* controller,
                           double max, voss::Flags flags) {
	Pose pose_target = Pose{target.x, target.y, 361};
	this->move(pose_target, controller, max, flags);
}

void AbstractChassis::move(Pose target,
                           controller::AbstractController* controller,
                           double max, voss::Flags flags) {
	// this->m.take();

	controller->set_target(target, flags & voss::Flags::RELATIVE);

	this->move_task(controller, max, flags);
}

void AbstractChassis::turn(double target, double max, voss::Flags flags) {
	this->turn(target, this->default_controller, max, flags);
}

void AbstractChassis::turn(double target,
                           controller::AbstractController* controller,
                           double max, voss::Flags flags) {
	// this->m.take();

	controller->set_target({0, 0, 0}, false);
	controller->set_angular_target(target, flags & voss::Flags::RELATIVE);

	this->turn_task(controller, max, flags);
}

void AbstractChassis::turn_to(Point target, double max, voss::Flags flags) {
	this->turn_to(target, this->default_controller, max, flags);
}

void AbstractChassis::turn_to(Point target,
                              controller::AbstractController* controller,
                              double max, voss::Flags flags) {
	// this->m.take();

	controller->set_target({target.x, target.y, 361},
	                       flags & voss::Flags::RELATIVE);

	this->turn_task(controller, max, flags);
}

} // namespace voss::chassis