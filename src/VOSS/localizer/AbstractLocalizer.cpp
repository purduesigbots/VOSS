#include "voss/localizer/AbstractLocalizer.hpp"
#include "pros/llemu.hpp"
#include <cmath>

namespace voss::localizer {

AbstractLocalizer::AbstractLocalizer() {
  this->mtx = new pros::Mutex;
  this->pose = {0.0, 0.0, 0.0};
}

void AbstractLocalizer::begin_localization() {
	pros::Task localization_task([=]() {
		while (true) {
			std::unique_lock<pros::Mutex> lock(this->mtx);
			this->update();
			pros::lcd::clear_line(1);
    	pros::lcd::print(1, "Updating Localizer");
			lock.unlock();

			pros::delay(10);
		}
	});
}

void AbstractLocalizer::set_pose(Pose pose) {
	std::unique_lock<pros::Mutex> lock(this->mtx);
	this->pose = pose;
}

Pose AbstractLocalizer::get_pose() {
	std::unique_lock<pros::Mutex> lock(this->mtx);
	Pose ret = this->pose;
	return ret;
}

double AbstractLocalizer::get_orientation_rad() {
	std::unique_lock<pros::Mutex> lock(this->mtx);
	double ret = this->pose.theta;
	return ret;
}

double AbstractLocalizer::get_orientation_deg() {
	std::unique_lock<pros::Mutex> lock(this->mtx);
	double ret = this->pose.theta * 180 * M_1_PI;
	return ret;
}

Point AbstractLocalizer::get_position() {
	std::unique_lock<pros::Mutex> lock(this->mtx);
	Point ret{this->pose.x, this->pose.y};
	return ret;
}
} // namespace voss::localizer