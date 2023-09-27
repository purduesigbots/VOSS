#include "voss/localizer/AbstractLocalizer.hpp"
#include "pros/rtos.hpp"
#include <cmath>

namespace voss::localizer {

AbstractLocalizer::AbstractLocalizer() {
	this->pose = {0.0, 0.0, 0.0};
}

void AbstractLocalizer::begin_localization() {
	pros::Task localization_task([=]() {
		while (true) {
			if(this->mtx.try_lock()){
				this->update();
				this->mtx.unlock();
			}
			pros::delay(10);
		}
	});
}

void AbstractLocalizer::set_pose(Pose pose) {
	while(true) {
		if(this->mtx.try_lock()) {
			this->pose = pose;
			this->mtx.unlock();
			return;
		}
		pros::delay(10);
	}
}

Pose AbstractLocalizer::get_pose() {
	while(true) {
		if(this->mtx.try_lock()) {
			Pose ret = this->pose;
			this->mtx.unlock();
			return ret;
		}
		pros::delay(10);
	}
}

double AbstractLocalizer::get_orientation_rad() {
	while(true) {
		if(this->mtx.try_lock()) {
			double ret = this->pose.theta;
			this->mtx.unlock();
			return ret;
		}
		pros::delay(10);
	}
}

double AbstractLocalizer::get_orientation_deg() {
	while(true){
		if(this->mtx.try_lock()) {
			double ret = this->pose.theta * 180 * M_1_PI;
			this->mtx.unlock();
			return ret;
		}
		pros::delay(10);
	}
}

Point AbstractLocalizer::get_position() {
	while(true) {
		if(this->mtx.try_lock()) {
			Point ret = {this->pose.x, this->pose.y};
			this->mtx.unlock();
			return ret;
		}
		pros::delay(10);
	}
}
} // namespace voss::localizer