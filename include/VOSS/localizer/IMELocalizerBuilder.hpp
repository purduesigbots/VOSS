#pragma once

#include "IMELocalizer.hpp"

namespace voss::localizer {

class IMELocalizerBuilder {

	std::vector<int8_t> leftMotors, rightMotors, horizontalMotors;
	double lr_tpi, mid_tpi;
	double track_width;
	double middle_dist;
	int imu_port;


public:
	IMELocalizerBuilder();

	static IMELocalizerBuilder newBuilder();


	IMELocalizerBuilder& withleftMotors(std::vector<int8_t> m);
	IMELocalizerBuilder& withrightMotors(std::vector<int8_t> m);
	IMELocalizerBuilder& withhorizontalMotors(std::vector<int8_t> m);
	IMELocalizerBuilder& withMiddleTPI(double mid_tpi);
	IMELocalizerBuilder& withLeftRightTPI(double lr_tpi);
	IMELocalizerBuilder& withTrackWidth(double track_width);
	IMELocalizerBuilder& withMiddleDistance(double middle_dist);
	IMELocalizerBuilder& withIMU(int imu_port);

	

	std::shared_ptr<IMELocalizer> build();
};

} // namespace voss::localizer