#pragma once

#include "legs/localizer/AbstractLocalizer.hpp"
#include "pros/adi.hpp"
#include <memory>

namespace legs::localizer {

class ADILocalizer : public AbstractLocalizer {

private:
	double prev_left_pos, prev_right_pos, prev_middle_pos;
	Pose prev_pose;

	double left_right_tpi, middle_tpi;
	double track_width;
	double left_right_dist, middle_dist;

	std::unique_ptr<pros::ADIEncoder> left_encoder;
	std::unique_ptr<pros::ADIEncoder> right_encoder;
	std::unique_ptr<pros::ADIEncoder> middle_encoder;

public:
	ADILocalizer(int left, int right, int mid, double lr_tpi, double mid_tpi,
	             double track_width, double middle_dist);

	int getLeftEncoderValue();
	int getRightEncoderValue();
	int getMiddleEncoderValue();

	void update();
};

} // namespace legs::localizer