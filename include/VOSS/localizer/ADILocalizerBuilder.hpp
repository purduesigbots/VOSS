#pragma once

#include "ADILocalizer.hpp"

namespace voss::localizer {

class ADILocalizerBuilder {

	int left, right, mid;
	double lr_tpi, mid_tpi;
	double track_width;
	double middle_dist;

public:
	ADILocalizerBuilder();

	static ADILocalizerBuilder newBuilder();

	ADILocalizerBuilder& withLeftEncoder(int c);
	ADILocalizerBuilder& withRightEncoder(int c);
	ADILocalizerBuilder& withMiddleEncoder(int c);
	ADILocalizerBuilder& withLeftRightTPI(double lr_tpi);
	ADILocalizerBuilder& withMiddleTPI(double mid_tpi);
	ADILocalizerBuilder& withTrackWidth(double track_width);
	ADILocalizerBuilder& withMiddleDistance(double middle_dist);

	ADILocalizer build();
};

} // namespace voss::localizer