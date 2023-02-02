#pragma once

#include <vector>

#include "api.h"

namespace legs {

typedef enum {
	LEGS_ROTATION,
	LEGS_ADI_ENCODER,
	LEGS_MOT_ENCODER,
	LEGS_INVALID_ENC_TYPE
} distance_tracker_type;

class BaseDistanceTracker {
protected:
	BaseDistanceTracker() {
	}

public:
	double get_dist_traveled();
};

// class for DistanceTracker Class

class LegsDistanceTracker : BaseDistanceTracker {
public:
	LegsDistanceTracker(unsigned int port_num, legs::distance_tracker_type type,
	                    bool reverse);
	LegsDistanceTracker(unsigned int port_num, int expander_port, bool reverse,
	                    double tpi);
	double get_dist_traveled();
	void set_tpi(double tpi);

private:
	int port_num;
	legs::distance_tracker_type type;
	double tpi;
	std::shared_ptr<pros::ADIEncoder> enc;
	std::shared_ptr<pros::Rotation> rot;
	std::shared_ptr<pros::MotorGroup> mot;
};

} // namespace legs