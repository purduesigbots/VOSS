#include "legs/chassis/AbstractChassis.hpp"
#include "main.h"

namespace legs::chassis {

AbstractChassis::AbstractChassis(
    std::initializer_list<int8_t> left_motors,
    std::initializer_list<int8_t> right_motors,
    controller::AbstractController& default_controller) {
	this->left_motors = std::make_unique<pros::Motor_Group>(left_motors);
	this->right_motors = std::make_unique<pros::Motor_Group>(right_motors);
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
	controller->reset();
	while (!this->execute(controller->get_command(target))) {
		printf("here\n");
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