#include "legs/chassis/AbstractChassis.hpp"

namespace legs::chassis {

AbstractChassis::AbstractChassis(
    controller::AbstractController& default_controller) {
	this->default_controller = &default_controller;
}

void AbstractChassis::move(Point target) {
	this->move(target, this->default_controller);
}

void AbstractChassis::move(Pose target) {
	this->move(target, this->default_controller);
}

void AbstractChassis::move(Point target,
                           controller::AbstractController* controller) {

	Pose pose_target = Pose{target.x, target.y, 361};
	controller->reset();
	while (!this->execute(controller->get_command(pose_target))) {
		pros::delay(10);
	}
}

void AbstractChassis::move(Pose target,
                           controller::AbstractController* controller) {
	controller->reset();
	while (!this->execute(controller->get_command(target))) {
		pros::delay(10);
	}
}

} // namespace legs::chassis