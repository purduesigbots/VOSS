#include "voss/localizer/ADILocalizer.hpp"
#include "pros/adi.hpp"

#include <cmath>
#include <memory>

namespace voss::localizer {

ADILocalizer::ADILocalizer(int left, int right, int mid, double lr_tpi,
                           double mid_tpi, double track_width,
                           double middle_dist)
    : prev_left_pos(0.0), prev_right_pos(0.0), prev_middle_pos(0.0),
      left_right_tpi(lr_tpi), middle_tpi(mid_tpi), track_width(track_width),
      middle_dist(middle_dist) {

	this->left_right_dist = track_width / 2;

	this->left_encoder = nullptr;
	this->right_encoder = nullptr;
	this->middle_encoder = nullptr;

	if (left != 0)
		this->left_encoder = std::make_unique<pros::adi::Encoder>(
		    abs(left), abs(left) + 1, left < 0);

	if (right != 0)
		this->right_encoder = std::make_unique<pros::adi::Encoder>(
		    abs(right), abs(right) + 1, right < 0);

	if (mid != 0)
		this->middle_encoder =
		    std::make_unique<pros::adi::Encoder>(abs(mid), abs(mid) + 1, mid < 0);
}

int ADILocalizer::getLeftEncoderValue() {
	if (left_encoder) {
		return this->left_encoder->get_value();
	} else {
		return 0.0;
	}
}

int ADILocalizer::getRightEncoderValue() {
	if (right_encoder) {
		return this->right_encoder->get_value();
	} else {
		return 0.0;
	}
}

int ADILocalizer::getMiddleEncoderValue() {
	if (middle_encoder) {
		return this->middle_encoder->get_value();
	} else {
		return 0.0;
	}
}

void ADILocalizer::update() {
	double left_pos = getLeftEncoderValue();
	double right_pos = getRightEncoderValue();
	double middle_pos = getMiddleEncoderValue();

	double delta_left = 0.0;
	if (left_encoder)
		delta_left = (left_pos - prev_left_pos) / left_right_tpi;

	double delta_right = 0.0;
	if (right_encoder)
		delta_right = (right_pos - prev_right_pos) / left_right_tpi;

	double delta_middle = 0.0;
	if (middle_encoder)
		delta_middle = (middle_pos - prev_middle_pos) / middle_tpi;

	double delta_angle = 0.0;
	if (left_encoder || right_encoder)
		delta_angle = (delta_right - delta_left) / track_width;

	this->pose.theta += delta_angle;

	prev_left_pos = left_pos;
	prev_right_pos = right_pos;
	prev_middle_pos = middle_pos;
	prev_pose = pose;

	double local_x;
	double local_y;

	if (delta_angle) {
		double i = sin(delta_angle / 2.0) * 2.0;
		local_x = (delta_right / delta_angle - left_right_dist) * i;
		local_y = (delta_middle / delta_angle + middle_dist) * i;
	} else {
		local_x = delta_right;
		local_y = delta_middle;
	}

	double p = this->pose.theta - delta_angle / 2.0; // global angle

	// convert to absolute displacement
	this->pose.x += cos(p) * local_x - sin(p) * local_y;
	this->pose.y += sin(p) * local_x + cos(p) * local_y;
}

} // namespace voss::localizer