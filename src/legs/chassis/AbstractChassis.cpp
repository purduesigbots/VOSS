#include "legs/chassis/AbstractChassis.hpp"
#include "pros/rtos.hpp"

namespace legs::chassis {

AbstractChassis::AbstractChassis(
    controller::AbstractController& default_controller) {
	this->default_controller = &default_controller;
}

void AbstractChassis::move(Point target, uint8_t flags) {
	this->move(target, this->default_controller, flags);
}

void AbstractChassis::move(Pose target, uint8_t flags) {
	this->move(target, this->default_controller, flags);
}

void AbstractChassis::move(Point target,
                           controller::AbstractController* controller,
                           uint8_t flags) {

	Pose pose_target = Pose{target.x, target.y, 361};
	this->move(pose_target, controller, flags);
}

void AbstractChassis::move(Pose target,
                           controller::AbstractController* controller,
                           uint8_t flags) {

	this->m.take();

	controller->set_target(target, flags & legs::RELATIVE);

	pros::Task running_t([=]() {
		controller->reset();
		while (!this->execute(
		    controller->get_command(flags & legs::REVERSE, flags & legs::THRU))) {
			pros::delay(10);
		}
		this->m.give();
	});

	if (flags & legs::ASYNC) {
		return;
	}

	this->m.take();
	this->m.give();
}

} // namespace legs::chassis