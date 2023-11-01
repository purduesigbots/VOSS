#include "voss/localizer/ADILocalizerBuilder.hpp"
#include <unordered_set>

namespace voss::localizer {

ADILocalizerBuilder::ADILocalizerBuilder()
    : left(0), right(0), mid(0), lr_tpi(0), mid_tpi(0), track_width(0) {
}

ADILocalizerBuilder ADILocalizerBuilder::newBuilder() {
	ADILocalizerBuilder builder;
	return builder;
}

ADILocalizerBuilder& ADILocalizerBuilder::withLeftEncoder(int c) {
	this->left = c;
	return *this;
}

ADILocalizerBuilder& ADILocalizerBuilder::withRightEncoder(int c) {
	this->right = c;
	return *this;
}

ADILocalizerBuilder& ADILocalizerBuilder::withMiddleEncoder(int c) {
	this->mid = c;
	return *this;
}

ADILocalizerBuilder& ADILocalizerBuilder::withLeftRightTPI(double lr_tpi) {
	this->lr_tpi = lr_tpi;
	return *this;
}

ADILocalizerBuilder& ADILocalizerBuilder::withMiddleTPI(double mid_tpi) {
	this->mid_tpi = mid_tpi;
	return *this;
}

ADILocalizerBuilder& ADILocalizerBuilder::withTrackWidth(double track_width) {
	this->track_width = track_width;
	return *this;
}

ADILocalizerBuilder&
ADILocalizerBuilder::withMiddleDistance(double middle_dist) {
	this->middle_dist = middle_dist;
	return *this;
}

std::shared_ptr<ADILocalizer> ADILocalizerBuilder::build() {
	std::unordered_set<unsigned char> valid_representations = {
	    0b11111111, 0b11011111, 0b11011101, 0b11010111, 0b11010101, 0b10111111,
	    0b10011111, 0b10011101, 0b10010111, 0b10010101, 0b1111111,  0b1011111,
	    0b1011101,  0b1010111,  0b1010101,  0b111111,   0b111011,   0b101111,
	    0b101011,   0b11111,    0b11101,    0b11011,    0b11001,    0b10111,
	    0b10101,    0b10011,    0b10001,    0b1111,     0b1101,     0b1011,
	    0b1001,     0b111,      0b101,      0b11,       0b1,        0b11111110,
	    0b11011110, 0b11011100, 0b11010110, 0b11010100, 0b10111110, 0b10011110,
	    0b10011100, 0b10010110, 0b10010100, 0b1111110,  0b1011110,  0b1011100,
	    0b1010110,  0b1010100,  0b111110,   0b111010,   0b101110,   0b101010,
	    0b11110,    0b11100,    0b11010,    0b11000,    0b10110,    0b10100,
	    0b10010,    0b10000,    0b1110,     0b1100,     0b1010,     0b1000,
	    0b110,      0b100,      0b10,       0b0};

	unsigned char rep = 0;

	rep |= (left != 0) << 7;
	rep |= (right != 0) << 6;
	rep |= (lr_tpi > 0) << 5;
	rep |= (mid != 0) << 4;
	rep |= (mid_tpi > 0) << 3;
	rep |= (track_width > 0) << 2;
	rep |= (middle_dist > 0) << 1;
	// rep |= (imu != 0) << 0;

	if (valid_representations.find(rep) == valid_representations.end()) {
		return nullptr;
	} else {
		return std::make_shared<ADILocalizer>(left, right, mid, lr_tpi, mid_tpi,
		                                      track_width, middle_dist);
	}
}

} // namespace voss::localizer