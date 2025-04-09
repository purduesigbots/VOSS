#include "VOSS/localizer/TrackingWheelLocalizer.hpp"

#include "VOSS/constants.hpp"
#include "VOSS/utils/angle.hpp"
#include "VOSS/utils/math.hpp"
#include <utility>

namespace voss::localizer {

TrackingWheelLocalizer::TrackingWheelLocalizer(
    std::unique_ptr<AbstractTrackingWheel> left,
    std::unique_ptr<AbstractTrackingWheel> right,
    std::unique_ptr<AbstractTrackingWheel> middle,
    std::vector<std::unique_ptr<pros::IMU>> imu, double left_right_dist,
    double middle_dist, Pose offset)
    : AbstractLocalizer(), left_tracking_wheel(std::move(left)),
      right_tracking_wheel(std::move(right)),
      middle_tracking_wheel(std::move(middle)), imu(std::move(imu)),
      left_right_dist(left_right_dist), middle_dist(middle_dist),
      prev_left_pos(0.0), prev_right_pos(0.0), prev_middle_pos(0.0),
      offset({offset.x, offset.y, offset.theta.value_or(0.0)}) {
    this->left_right_dist += offset.y;
    this->middle_dist += offset.x;
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
    if (!imu.empty()) {
        std::vector<double> rotations;
        for (const auto& s_imu : this->imu) {
            if (s_imu->is_installed()) {
                rotations.push_back(s_imu->get_rotation());
            }
        }
        double rotation_avg = voss::get_avg(rotations);
        pose.theta = -to_radians(rotation_avg);
        delta_angle = real_pose.theta - prev_pose.theta;
        real_pose.theta = pose.theta.load();
    } else {
        delta_angle = (delta_right - delta_left) / (2 * left_right_dist);
        pose.theta += delta_angle;
    }

    prev_left_pos += delta_left;
    prev_right_pos += delta_right;
    prev_middle_pos += delta_middle;
    prev_pose = real_pose;

    double local_x;
    double local_y;

    if (delta_angle) {
        double i = sin(delta_angle / 2.0) * 2.0;
        if (left_tracking_wheel && right_tracking_wheel) {
            local_x = (delta_right + delta_left) / (2 * delta_angle) * i;
        } else if (right_tracking_wheel) {
            local_x = (delta_right / delta_angle - left_right_dist) * i;
        } else if (left_tracking_wheel) {
            local_x = (delta_left / delta_angle + left_right_dist) * i;
        }
        local_y = (delta_middle / delta_angle + middle_dist) * i;
    } else {
        local_x = delta_right;
        local_y = delta_middle;
    }

    double p =
        (0 * -M_PI_4) + this->pose.theta - delta_angle / 2.0; // global angle

    // convert to absolute displacement
    this->real_pose.x += cos(p) * local_x - sin(p) * local_y;
    this->real_pose.y += sin(p) * local_x + cos(p) * local_y;

    this->pose.x = real_pose.x + horizontal_offset * cos(this->pose.theta);
    this->pose.y = real_pose.y + horizontal_offset * sin(this->pose.theta);
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
    if (!imu.empty()) {
        // check if IMU is installed
        // otherwise remove it from the list
        std::erase_if(imu,
                      [](const auto& s_imu) { return !s_imu->is_installed(); });

        // calibrate imu without blocking should speed up the calibration
        // process
        for (auto& s_imu : this->imu) {
            s_imu->reset(true);
        }
    }
    this->pose = AtomicPose{0, 0, 0.0};
}

void TrackingWheelLocalizer::set_pose(Pose pose) {
    std::unique_lock<pros::Mutex> lock(this->mtx);
    if (pose.theta.has_value()) {
        this->pose =
            AtomicPose{pose.x, pose.y, voss::to_radians(pose.theta.value())};
    } else {
        double h = this->pose.theta;
        this->pose = AtomicPose{pose.x, pose.y, h};
    }
    this->real_pose.x =
        this->pose.x - horizontal_offset * cos(this->pose.theta);
    this->real_pose.y =
        this->pose.y - horizontal_offset * sin(this->pose.theta);
    this->real_pose.theta = this->pose.theta.load();
    this->prev_pose = this->real_pose;
    if (!this->imu.empty() && pose.theta.has_value()) {
        for (const auto& s_imu : this->imu) {
            if (s_imu->is_installed()) {
                s_imu->set_rotation(-pose.theta.value());
            }
        }
    }
}

void TrackingWheelLocalizer::set_pose(double x, double y, double theta) {
    this->set_pose({x, y, theta});
}

void TrackingWheelLocalizer::set_horizontal_offset(double horizontal_offset) {
    this->horizontal_offset = horizontal_offset;
}

} // namespace voss::localizer
