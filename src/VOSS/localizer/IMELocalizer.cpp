#include "voss/localizer/IMELocalizer.hpp"
#include "AbstractLocalizer.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"

#include <cmath>
#include <memory>

namespace voss::localizer {

IMELocalizer::IMELocalizer(std::vector<int8_t> leftMotorsPorts, std::vector<int8_t> rightMotorsPorts, std::vector<int8_t> horizontalMotorsPorts, double lr_tpi, double mid_tpi, double track_width, double middle_dist, int imu_port)
    : prev_left_pos(0.0), prev_right_pos(0.0), prev_middle_pos(0.0),
      left_right_tpi(lr_tpi), middle_tpi(mid_tpi), track_width(track_width),
      middle_dist(middle_dist), imu_ports(imu_port) {

	this->left_right_dist = track_width / 2;
	this->leftMotors = nullptr;
	this->rightMotors = nullptr;
	this->horizontalMotors = nullptr;
	this->imu = nullptr;

    if (leftMotorsPorts.size() > 0) {
        this->leftMotors = std::make_unique<pros::MotorGroup>(leftMotorsPorts);
    }
    if (rightMotorsPorts.size() >0) {
        this->rightMotors = std::make_unique<pros::MotorGroup>(rightMotorsPorts);
    }
    if (horizontalMotorsPorts.size() > 0) {
        this->horizontalMotors = std::make_unique<pros::MotorGroup>(horizontalMotorsPorts);
    }
	if (imu_port != 0) {
        this->imu = std::make_unique<pros::IMU>(imu_port);
    }
	
}

double IMELocalizer::getLeftEncoderValue() {
	if (leftMotors) {
		return this->leftMotors->get_position();
	} else {
		errno = EIO;
		return PROS_ERR;
	}
}

double IMELocalizer::getRightEncoderValue() {
	if (rightMotors) {
		return this->rightMotors->get_position();
	} else {
		errno = EIO;
		return PROS_ERR;
	}
}

double IMELocalizer::getMiddleEncoderValue() {
	if (horizontalMotors) {
		return this->horizontalMotors->get_position();
	} else {
		errno = EIO;
		return PROS_ERR;
	}
}

double IMELocalizer::getIMUValue() {
	if (imu) {
		return this->imu->get_rotation()*(M_PI / 180.0);
	} else {
		errno = EIO;
		return PROS_ERR;
	}
}
void IMELocalizer::calibrate(){
	if (imu) {
		this->imu->reset();
		while(imu->is_calibrating()){
			pros::delay(10);
		}
	}
	if(leftMotors){
		leftMotors->tare_position();
	}
	if(rightMotors){
		rightMotors->tare_position();
	}
	if(horizontalMotors){
		horizontalMotors->tare_position();
	}
	this->pose = {0.0, 0.0, 0.0};
}

void IMELocalizer::update() {
	double left_pos = getLeftEncoderValue();
	double right_pos = getRightEncoderValue();
	double middle_pos = getMiddleEncoderValue();
	double imu_Value = getIMUValue();

	double delta_left = (left_pos - prev_left_pos) / left_right_tpi;
	double delta_right = (right_pos - prev_right_pos) / left_right_tpi;
	double delta_middle = (middle_pos - prev_middle_pos) / middle_tpi;
	double delta_angle = 0.0;

	if (imu){
		this->pose.theta = -1 * imu_Value;
	}
	else{
		delta_angle = (delta_right - delta_left) / track_width;
		this->pose.theta += delta_angle;
	}

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