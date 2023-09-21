#include "voss/localizer/AbstractLocalizer.hpp"
#include "pros/rtos.hpp"
#include <cmath>

namespace voss::localizer {

AbstractLocalizer::AbstractLocalizer() {
	this->mtx = false;
	this->pose = {0.0, 0.0, 0.0};
}

void AbstractLocalizer::begin_localization() {
	pros::Task localization_task([=]() {
		while (true) {
			while (this->mtx)
				pros::delay(10);

			this->mtx = true;
			this->update();
			this->mtx = false;

			pros::delay(10);
		}
	});
}

void AbstractLocalizer::set_pose(Pose pose) {
	while (this->mtx)
		pros::delay(10);
	this->mtx = true;
	this->pose = pose;
	this->mtx = false;
}

Pose AbstractLocalizer::get_pose() {
	while (this->mtx)
		pros::delay(10);
	this->mtx = true;
	Pose ret = this->pose;
	this->mtx = false;

	return ret;
}

double AbstractLocalizer::get_orientation_rad() {
	while (this->mtx)
		pros::delay(10);
	this->mtx = true;
	double ret = this->pose.theta;
	this->mtx = false;

	return ret;
}

double AbstractLocalizer::get_orientation_deg() {
	while (this->mtx)
		pros::delay(10);
	this->mtx = true;
	double ret = this->pose.theta * 180 * M_1_PI;
	this->mtx = false;

	return ret;
}

Point AbstractLocalizer::get_position() {
	while (this->mtx)
		pros::delay(10);
	this->mtx = true;
	Point ret{this->pose.x, this->pose.y};
	this->mtx = false;

	return ret;
}
} // namespace voss::localizer