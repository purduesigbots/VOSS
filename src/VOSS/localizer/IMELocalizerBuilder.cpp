#include "voss/localizer/IMELocalizerBuilder.hpp"
#include "IMELocalizer.hpp"
#include "pros/adi.h"
#include <memory>
#include <unordered_set>

namespace voss::localizer {

IMELocalizerBuilder::IMELocalizerBuilder()
    : leftMotors({}), rightMotors({}), horizontalMotors({}), lr_tpi(0),
      mid_tpi(0), track_width(0), imu_port(0) {
}

IMELocalizerBuilder IMELocalizerBuilder::newBuilder() {
	IMELocalizerBuilder builder;
	return builder;
}

IMELocalizerBuilder&
IMELocalizerBuilder::withleftMotors(std::vector<int8_t> m) {
	this->leftMotors = m;
	return *this;
}

IMELocalizerBuilder&
IMELocalizerBuilder::withrightMotors(std::vector<int8_t> m) {
	this->rightMotors = m;
	return *this;
}

IMELocalizerBuilder&
IMELocalizerBuilder::withhorizontalMotors(std::vector<int8_t> m) {
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
IMELocalizerBuilder& IMELocalizerBuilder::withIMU(int imu_port) {
	this->imu_port = imu_port;
	return *this;
}

std::shared_ptr<IMELocalizer> IMELocalizerBuilder::build() {

	std::unordered_set<unsigned char> valid_representations = {
	    0b11111111, 0b11110111, 0b11110101, 0b11110011, 0b11110001, 0b10111111,
	    0b10110111, 0b10110101, 0b10110011, 0b10110001, 0b1111111,  0b1110111,
	    0b1110101,  0b1110011,  0b1110001,  0b111111,   0b101111,   0b11111,
	    0b1111,     0b110111,   0b110101,   0b100111,   0b100101,   0b110011,
	    0b110001,   0b100011,   0b100001,   0b10111,    0b10101,    0b111,
	    0b101,      0b10011,    0b10001,    0b11,       0b1,        0b11111110,
	    0b11110110, 0b11110100, 0b11110010, 0b11110000, 0b10111110, 0b10110110,
	    0b10110100, 0b10110010, 0b10110000, 0b1111110,  0b1110110,  0b1110100,
	    0b1110010,  0b1110000,  0b111110,   0b101110,   0b11110,    0b1110,
	    0b110110,   0b110100,   0b100110,   0b100100,   0b110010,   0b110000,
	    0b100010,   0b100000,   0b10110,    0b10100,    0b110,      0b100,
	    0b10010,    0b10000,    0b10,       0b0};

	unsigned char rep = 0;

	rep |= (leftMotors.size() != 0) << 7;
	rep |= (rightMotors.size() != 0) << 6;
	rep |= (lr_tpi > 0) << 5;
	rep |= (track_width > 0) << 4;
	rep |= (horizontalMotors.size() != 0) << 3;
	rep |= (mid_tpi > 0) << 2;
	rep |= (middle_dist > 0) << 1;
	rep |= (imu_port != 0) << 0;
	
	if (valid_representations.find(rep) == valid_representations.end()) {
		return nullptr;
	}

	return std::make_shared<IMELocalizer>(leftMotors, rightMotors,
	                                      horizontalMotors, lr_tpi, mid_tpi,
	                                      track_width, middle_dist, imu_port);
}

} // namespace voss::localizer