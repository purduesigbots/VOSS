#include "voss/chassis/AbstractChassis.hpp"
#include "pros/rtos.hpp"

namespace voss::chassis {

AbstractChassis::AbstractChassis(
    controller::AbstractController& default_controller) {
	this->default_controller = &default_controller;
}

void AbstractChassis::move(Point target, double max, uint8_t flags) {
	this->move(target, this->default_controller, flags);
}

void AbstractChassis::move(Pose target, double max, uint8_t flags) {
	this->move(target, this->default_controller, flags);
}

void AbstractChassis::move(Point target,
                           controller::AbstractController* controller,
                           double max, uint8_t flags) {

	Pose pose_target = Pose{target.x, target.y, 361};
	this->move(pose_target, controller, flags);
}

void AbstractChassis::move(Pose target,
                           controller::AbstractController* controller,
                           double max, uint8_t flags) {

	this->m.take();

	controller->set_target(target, flags & voss::RELATIVE);

	pros::Task running_t([=]() {
		controller->reset();
		while (!this->execute(
		    controller->get_command(flags & voss::REVERSE, flags & voss::THRU),
		    max)) {
			pros::delay(10);
		}
		this->m.give();
	});

	if (flags & voss::ASYNC) {
		return;
	}

	this->m.take();
	this->m.give();
}

} // namespace voss::chassis