#pragma once

#include "legs/utils/Point.hpp"
#include "legs/utils/Pose.hpp"

namespace legs::localizer {

class AbstractLocalizer {
protected:
	bool mtx;
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
};

} // namespace legs::localizer