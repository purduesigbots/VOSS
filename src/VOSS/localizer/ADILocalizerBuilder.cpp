#include "voss/localizer/ADILocalizerBuilder.hpp"

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

ADILocalizer ADILocalizerBuilder::build() {
	return ADILocalizer(left, right, mid, lr_tpi, mid_tpi, track_width, middle_dist);
}

} // namespace voss::localizer