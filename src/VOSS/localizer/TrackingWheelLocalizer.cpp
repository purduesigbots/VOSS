#include "VOSS/localizer/TrackingWheelLocalizer.hpp"
#include "VOSS/utils/angle.hpp"

namespace voss::localizer {

TrackingWheelLocalizer::TrackingWheelLocalizer(
    std::unique_ptr<AbstractTrackingWheel> left,
    std::unique_ptr<AbstractTrackingWheel> right,
    std::unique_ptr<AbstractTrackingWheel> middle,
    std::unique_ptr<pros::IMU> imu, double left_right_dist, double middle_dist)
    : AbstractLocalizer(), left_tracking_wheel(std::move(left)),
      right_tracking_wheel(std::move(right)),
      middle_tracking_wheel(std::move(middle)), imu(std::move(imu)),
      left_right_dist(left_right_dist), middle_dist(middle_dist),
      prev_left_pos(0.0), prev_right_pos(0.0), prev_middle_pos(0.0) {
}

void TrackingWheelLocalizer::update() {
    double delta_left = 0.0;
    double delta_right = 0.0;
    double delta_middle = 0.0;
    double delta_angle = 0.0;

    if (left_tracking_wheel) {
        delta_left = left_tracking_wheel->get_dist_travelled() - prev_left_pos;
    }
    if (right_tracking_wheel) {
        delta_right =
            right_tracking_wheel->get_dist_travelled() - prev_right_pos;
    }
    if (middle_tracking_wheel) {
        delta_middle =
            middle_tracking_wheel->get_dist_travelled() - prev_middle_pos;
    }
    if (imu) {
        pose.theta = -to_radians(imu->get_rotation());
        delta_angle = pose.theta - prev_pose.theta;
    } else {
        delta_angle = (delta_right - delta_left) / (2 * left_right_dist);
        pose.theta += delta_angle;
    }

    prev_left_pos += delta_left;
    prev_right_pos += delta_right;
    prev_middle_pos += delta_middle;
    prev_pose = pose;

    double local_x;
    double local_y;

    if (delta_angle) {
        double i = sin(delta_angle / 2.0) * 2.0;
        if (right_tracking_wheel) {
            local_x = (delta_right / delta_angle - left_right_dist) * i;
        } else if (left_tracking_wheel) {
            local_x = (delta_left / delta_angle + left_right_dist) * i;
        }
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

void TrackingWheelLocalizer::calibrate() {
    if (left_tracking_wheel) {
        left_tracking_wheel->reset();
    }
    if (right_tracking_wheel) {
        right_tracking_wheel->reset();
    }
    if (middle_tracking_wheel) {
        middle_tracking_wheel->reset();
    }
    if (imu) {
        imu->reset(true);
    }
    this->pose = {0.0, 0.0, 0.0};
}

void TrackingWheelLocalizer::set_pose(Pose pose) {
    this->AbstractLocalizer::set_pose(pose);
    this->prev_pose = this->pose;
    if (this->imu) {
        this->imu->set_rotation(-pose.theta);
    }
}

void TrackingWheelLocalizer::set_pose(double x, double y, double theta) {
    this->set_pose({x, y, theta});
}

} // namespace voss::localizer