#include "voss/localizer/IMELocalizerBuilder.hpp"

namespace voss::localizer {

IMELocalizerBuilder::IMELocalizerBuilder()
    : leftMotors({}), rightMotors({}), horizontalMotors({}), lr_tpi(0), mid_tpi(0), track_width(0) {
}

IMELocalizerBuilder IMELocalizerBuilder::newBuilder() {
	IMELocalizerBuilder builder;
	return builder;
}

IMELocalizerBuilder& IMELocalizerBuilder::withleftMotors(std::vector<int8_t> m) {
	this->leftMotors = m;
	return *this;
}

IMELocalizerBuilder& IMELocalizerBuilder::withrightMotors(std::vector<int8_t> m) {
	this->rightMotors = m;
	return *this;
}

IMELocalizerBuilder& IMELocalizerBuilder::withhorizontalMotors(std::vector<int8_t> m) {
	this->horizontalMotors = m;
	return *this;
}

IMELocalizerBuilder& IMELocalizerBuilder::withLeftRightTPI(double lr_tpi) {
	this->lr_tpi = lr_tpi;
	return *this;
}

IMELocalizerBuilder& IMELocalizerBuilder::withMiddleTPI(double mid_tpi) {
	this->mid_tpi = mid_tpi;
	return *this;
}

IMELocalizerBuilder& IMELocalizerBuilder::withTrackWidth(double track_width) {
	this->track_width = track_width;
	return *this;
}

IMELocalizerBuilder&
IMELocalizerBuilder::withMiddleDistance(double middle_dist) {
	this->middle_dist = middle_dist;
	return *this;
}

IMELocalizer IMELocalizerBuilder::build() {
	IMELocalizer l(leftMotors, rightMotors, horizontalMotors, lr_tpi, mid_tpi, track_width, middle_dist);
	return l;
}

} // namespace voss::localizer