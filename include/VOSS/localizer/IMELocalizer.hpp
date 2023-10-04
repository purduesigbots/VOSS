#pragma once

#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "voss/localizer/AbstractLocalizer.hpp"
#include <memory>


namespace voss::localizer {

class IMELocalizer : public AbstractLocalizer {

private:
	double prev_left_pos, prev_right_pos, prev_middle_pos;
	Pose prev_pose;

	double left_right_tpi, middle_tpi;
	double track_width;
	double left_right_dist, middle_dist;

    std::unique_ptr<pros::MotorGroup> leftMotors;
    std::unique_ptr<pros::MotorGroup> rightMotors;
    std::unique_ptr<pros::MotorGroup> horizontalMotors;


public:
	IMELocalizer(std::vector<int8_t> leftMotorsPorts, std::vector<int8_t> rightMotorsPorts, std::vector<int8_t> horizontalMotor, double lr_tpi, double mid_tpi, double track_width, double middle_dist);
    
    int getLeftEncoderValue();
	int getRightEncoderValue();
	int getMiddleEncoderValue();

	void update();
};

} // namespace voss::localizer