#pragma once

#include "pros/rtos.hpp"
#include "voss/utils/Point.hpp"
#include "voss/utils/Pose.hpp"
#include <mutex>


namespace voss::localizer {

class AbstractLocalizer {
protected:
	pros::Mutex mtx;
	Pose pose;

public:
	AbstractLocalizer();

	virtual void update() = 0;
	void begin_localization();

	void set_pose(Pose pose);

	Pose get_pose();
	double get_orientation_rad();
	double get_orientation_deg();
	Point get_position();
	virtual void calibrate() = 0;
};

} // namespace voss::localizer