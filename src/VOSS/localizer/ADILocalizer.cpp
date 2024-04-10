#include "VOSS/localizer/ADILocalizer.hpp"
#include "ADILocalizer.hpp"
#include "pros/adi.hpp"
#include "VOSS/constants.hpp"

#include <cmath>
#include <memory>

namespace voss::localizer {

// Creating a localizer object with varibale option based on adi imput sensors
ADILocalizer::ADILocalizer(int left, int right, int mid, double lr_tpi,
                           double mid_tpi, double track_width,
                           double middle_dist, int imu_port)
    : prev_left_pos(0.0), prev_right_pos(0.0), prev_middle_pos(0.0),
      left_right_tpi(lr_tpi), middle_tpi(mid_tpi), track_width(track_width),
      middle_dist(middle_dist), imu_ports(imu_port) {

    this->left_right_dist = track_width / 2;

    this->left_encoder = nullptr;
    this->right_encoder = nullptr;
    this->middle_encoder = nullptr;
    this->imu = nullptr;

    if (left != 0)
        this->left_encoder = std::make_unique<pros::adi::Encoder>(
            abs(left), abs(left) + 1, left < 0);

    if (right != 0)
        this->right_encoder = std::make_unique<pros::adi::Encoder>(
            abs(right), abs(right) + 1, right < 0);

    if (mid != 0)
        this->middle_encoder = std::make_unique<pros::adi::Encoder>(
            abs(mid), abs(mid) + 1, mid < 0);
    if (imu_port != 0)
        this->imu = std::make_unique<pros::IMU>(imu_port);
}

double ADILocalizer::get_left_encoder_value() {
    if (left_encoder) {
        return this->left_encoder->get_value();
    } else {
        return 0.0;
    }
}

double ADILocalizer::get_right_encoder_value() {
    if (right_encoder) {
        return this->right_encoder->get_value();
    } else {
        return 0.0;
    }
}

double ADILocalizer::get_middle_encoder_value() {
    if (middle_encoder) {
        return this->middle_encoder->get_value();
    } else {
        return 0.0;
    }
}

double ADILocalizer::get_imu_value() {
    if (imu) {
        return this->imu->get_rotation() * (M_PI / 180.0);
    } else {
        errno = EIO;
        return PROS_ERR;
    }
}

void ADILocalizer::calibrate() {
    if (left_encoder) {
        this->left_encoder->reset();
    }
    if (right_encoder) {
        this->right_encoder->reset();
    }
    if (middle_encoder) {
        this->middle_encoder->reset();
    }
    if (imu) {
        this->imu->reset();
        while (this->imu->is_calibrating()) {
            pros::delay(constants::SENSOR_UPDATE_DELAY);
        }
    }
    this->pose = voss::AtomicPose{0.0, 0.0, 0.0};
}

// Calculates the current position of the robot
// Uses the change in value of the encoders to calculate the change in position
// If no imu is present, the robot's heading is calculated using the difference
// in the left and right encoder values Angle is the differnce between the robot
// heading and the global angle
void ADILocalizer::update() {
    double left_pos = get_left_encoder_value();
    double right_pos = get_right_encoder_value();
    double middle_pos = get_middle_encoder_value();
    double imu_value = get_imu_value();

    double delta_left = 0.0;
    if (left_encoder)
        delta_left = (left_pos - prev_left_pos) / left_right_tpi;

    double delta_right = 0.0;
    if (right_encoder)
        delta_right = (right_pos - prev_right_pos) / left_right_tpi;

    double delta_middle = 0.0;
    if (middle_encoder)
        delta_middle = (middle_pos - prev_middle_pos) / middle_tpi;

    double delta_angle = 0.0;

    if (imu) {
        this->pose.theta = -1 * imu_value;
    } else {
        // if (left_encoder || right_encoder){
        delta_angle = (delta_right - delta_left) / track_width;
        this->pose.theta += delta_angle;
        //}
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

void ADILocalizer::set_pose(Pose pose) {
    this->AbstractLocalizer::set_pose(pose);
    if (this->imu && pose.theta.has_value()) {
        this->imu->set_rotation(-pose.theta.value());
    }
}
void ADILocalizer::set_pose(double x, double y, double theta) {
    this->set_pose({x, y, theta});
}

} // namespace voss::localizer