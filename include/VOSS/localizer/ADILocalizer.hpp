#pragma once

#include "pros/adi.hpp"
#include "voss/localizer/AbstractLocalizer.hpp"
#include <memory>
#include <atomic>


namespace voss::localizer {

class ADILocalizer : public AbstractLocalizer {

private:
	std::atomic<double> prev_left_pos, prev_right_pos, prev_middle_pos;
	AtomicPose prev_pose;

	std::atomic<double> left_right_tpi, middle_tpi;
	std::atomic<double> track_width;
	std::atomic<double> left_right_dist, middle_dist;

	std::unique_ptr<pros::adi::Encoder> left_encoder;
	std::unique_ptr<pros::adi::Encoder> right_encoder;
	std::unique_ptr<pros::adi::Encoder> middle_encoder;

public:
	ADILocalizer(int left, int right, int mid, double lr_tpi, double mid_tpi,
	             double track_width, double middle_dist);

	int getLeftEncoderValue();
	int getRightEncoderValue();
	int getMiddleEncoderValue();

	void update();
};

} // namespace voss::localizer