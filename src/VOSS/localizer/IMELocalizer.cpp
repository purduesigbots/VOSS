#include "VOSS/localizer/IMELocalizer.hpp"
#include "AbstractLocalizer.hpp"
#include "IMELocalizer.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"

#include <cmath>
#include <memory>

namespace voss::localizer {

// Creating a localizer object with varibale option based on internal motor
// encoders
IMELocalizer::IMELocalizer(std::vector<int8_t> left_motors_ports,
                           std::vector<int8_t> right_motors_ports,
                           std::vector<int8_t> horizontal_motors_ports,
                           double lr_tpi, double mid_tpi, double track_width,
                           double middle_dist, int imu_port)
    : prev_left_pos(0.0), prev_right_pos(0.0), prev_middle_pos(0.0),
      left_right_tpi(lr_tpi), middle_tpi(mid_tpi), track_width(track_width),
      middle_dist(middle_dist), imu_ports(imu_port) {

    this->left_right_dist = track_width / 2;
    this->left_motors = nullptr;
    this->right_motors = nullptr;
    this->horizontal_motors = nullptr;
    this->imu = nullptr;

    if (left_motors_ports.size() > 0) {
        this->left_motors =
            std::make_unique<pros::MotorGroup>(left_motors_ports);
    }
    if (right_motors_ports.size() > 0) {
        this->right_motors =
            std::make_unique<pros::MotorGroup>(right_motors_ports);
    }
    if (horizontal_motors_ports.size() > 0) {
        this->horizontal_motors =
            std::make_unique<pros::MotorGroup>(horizontal_motors_ports);
    }
    if (imu_port != 0) {
        this->imu = std::make_unique<pros::IMU>(imu_port);
    }
}

double IMELocalizer::get_left_encoder_value() {
    if (left_motors) {
        return this->left_motors->get_position();
    } else {
        errno = EIO;
        return PROS_ERR;
    }
}

double IMELocalizer::get_right_encoder_value() {
    if (right_motors) {
        return this->right_motors->get_position();
    } else {
        errno = EIO;
        return PROS_ERR;
    }
}

double IMELocalizer::get_middle_encoder_value() {
    if (horizontal_motors) {
        return this->horizontal_motors->get_position();
    } else {
        errno = EIO;
        return PROS_ERR;
    }
}

double IMELocalizer::get_imu_value() {
    if (imu) {
        return this->imu->get_rotation() * (M_PI / 180.0);
    } else {
        errno = EIO;
        return PROS_ERR;
    }
}
void IMELocalizer::calibrate() {
    if (imu) {
        this->imu->reset();
        while (imu->is_calibrating()) {
            pros::delay(10);
        }
    }
    if (left_motors) {
        left_motors->tare_position();
    }
    if (right_motors) {
        right_motors->tare_position();
    }
    if (horizontal_motors) {
        horizontal_motors->tare_position();
    }
    this->pose = voss::AtomicPose{0.0, 0.0, 0.0};
}
// Calculates the current position of the robot
// Uses the change in value of the encoders to calculate the change in position
// If no imu is present, the robot's heading is calculated using the difference
// in the left and right encoder values Angle is the differnce between the robot
// heading and the global angle
void IMELocalizer::update() {
    double left_pos = get_left_encoder_value();
    double right_pos = get_right_encoder_value();
    double middle_pos = get_middle_encoder_value();
    double imu_Value = get_imu_value();

    double delta_left = 0.0;
    if (left_motors)
        delta_left = (left_pos - prev_left_pos) / left_right_tpi;

    double delta_right = 0.0;
    if (right_motors)
        delta_right = (right_pos - prev_right_pos) / left_right_tpi;

    double delta_middle = 0.0;
    if (horizontal_motors)
        delta_middle = (middle_pos - prev_middle_pos) / middle_tpi;

    double delta_angle = 0.0;
    if (imu) {
        this->pose.theta = -1 * imu_Value;
    } else {
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

void IMELocalizer::set_pose(Pose pose) {
    this->AbstractLocalizer::set_pose(pose);
    if (this->imu && pose.theta.has_value()) {
        this->imu->set_rotation(-pose.theta.value());
    }
}

} // namespace voss::localizer