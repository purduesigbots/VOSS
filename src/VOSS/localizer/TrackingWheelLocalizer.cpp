#include "VOSS/localizer/TrackingWheelLocalizer.hpp"
#include "Eigen/Dense"
#include "VOSS/constants.hpp"
#include "VOSS/utils/angle.hpp"
#include <algorithm>
#include <numeric>

namespace voss::localizer {

static constexpr double meter_to_inch = 39.3701;
static constexpr double inch_to_meter = 1.0 / meter_to_inch;

TrackingWheelLocalizer::TrackingWheelLocalizer(
    std::unique_ptr<AbstractTrackingWheel> left,
    std::unique_ptr<AbstractTrackingWheel> right,
    std::unique_ptr<AbstractTrackingWheel> middle,
    std::vector<std::shared_ptr<pros::IMU>> imus, double left_right_dist,
    double middle_dist)
    : AbstractLocalizer(), left_tracking_wheel(std::move(left)),
      right_tracking_wheel(std::move(right)),
      middle_tracking_wheel(std::move(middle)), imus(std::move(imus)),
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
    if (!imus.empty()) {
        double imu_avg =
            std::accumulate(
                imus.begin(), imus.end(), 0.0,
                [](double sum, const std::shared_ptr<pros::IMU>& imu) {
                    return sum + imu->get_rotation();
                }) /
            imus.size();
        double theta = -to_radians(imu_avg);
        //        pose.theta = -to_radians(imu_avg);
        delta_angle = theta - prev_theta;
    } else {
        delta_angle = (delta_right - delta_left) / (2 * left_right_dist);
        pose.theta += delta_angle;
    }

    prev_left_pos += delta_left;
    prev_right_pos += delta_right;
    prev_middle_pos += delta_middle;
    prev_theta += delta_angle;

    double local_x;
    double local_y;

    constexpr static double eps = 1e-6;
    if (std::abs(delta_angle) > eps) {
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

    if (kalman_filter && gps) {
        kalman_filter->predict({local_x, local_y}, delta_angle);

        if (this->enable_gps && gps->get_error() > 0) {
            auto gps_pose = gps->get_position_and_orientation();
            Eigen::Vector3d gps_pos = {
                gps_pose.x * meter_to_inch, gps_pose.y * meter_to_inch,
                voss::to_radians(-gps_pose.yaw + 180.f)};
            kalman_filter->update_gps_pose_and_error(
                gps_pos, gps->get_error() * meter_to_inch);
        }

        auto new_state = kalman_filter->get_state();
        this->pose = new_state;
    } else {
        std::cerr << "kalman filter or gps not inited, using pure odom only"
                  << std::endl;
        double p = this->pose.theta - delta_angle / 2.0; // global angle

        // convert to absolute displacement
        this->pose.x += cos(p) * local_x - sin(p) * local_y;
        this->pose.y += sin(p) * local_x + cos(p) * local_y;
    }
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
    for (const auto& imu : imus) {
        imu->reset(true);
        while (imu->is_calibrating()) {
            pros::delay(constants::SENSOR_UPDATE_DELAY);
        }
    }
    this->pose = AtomicPose{0.0, 0.0, 0.0};
}

void TrackingWheelLocalizer::set_pose(Pose new_pose) {
    // new pose have heading in degree, this->pose have heading in radian, bad
    // design but idgaf
    this->AbstractLocalizer::set_pose(new_pose);
    if (new_pose.theta.has_value()) {
        for (auto& imu : imus) {
            imu->set_rotation(-new_pose.theta.value());
        }
        this->prev_theta.store(this->pose.theta);
    }

    if (kalman_filter && gps) {
        gps->set_position(
            pose.x * inch_to_meter, pose.y * inch_to_meter,
            -new_pose.theta.value_or(voss::to_degrees(this->pose.theta)));
        kalman_filter->set_pose(this->pose);
    }
}

void TrackingWheelLocalizer::set_pose(double x, double y, double theta) {
    this->set_pose({x, y, theta});
}

} // namespace voss::localizer