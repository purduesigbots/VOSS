#pragma once

#include "SSOV/localizer/OdometryLocalizer.hpp"
#include "SSOV/localizer/AbstractTrackingWheel.hpp"
#include "SSOV/common/Math.hpp"
#include "pros/imu.hpp"

namespace ssov {

class TrackingWheelLocalizer: public OdometryLocalizer {
    private:
        double prev_left_pos, prev_right_pos, prev_middle_pos, prev_angle;
        std::shared_ptr<AbstractTrackingWheel> left, right, middle;
        std::shared_ptr<pros::IMU> imu;
        double left_right_dist, middle_dist;
    public:
        TrackingWheelLocalizer(std::shared_ptr<AbstractTrackingWheel> left,
                           std::shared_ptr<AbstractTrackingWheel> right,
                           std::shared_ptr<AbstractTrackingWheel> middle,
                           std::shared_ptr<pros::IMU> imu,
                           double left_right_dist, double middle_dist, Pose local_offset = {}, uint32_t update_time = 10):
            OdometryLocalizer(local_offset, update_time),
            left(std::move(left)), right(std::move(right)),
            middle(std::move(middle)), imu(std::move(imu)),
            left_right_dist(left_right_dist), middle_dist(middle_dist) {};

        void calibrate() override {
            if (left) left->reset();
            if (right) right->reset();
            if (middle) middle->reset();
            if (imu) {
                imu->reset(true);
            }
            prev_left_pos = 0;
            prev_right_pos = 0;
            prev_middle_pos = 0;
            prev_angle = -to_radians(imu->get_rotation());
        }

        Pose get_local_change() {
            double dx = 0;
            double dy = 0;
            double dtheta = 0;
            double left_pos = 0;
            double right_pos = 0;
            double middle_pos = 0;

            if (left) left_pos = left->get_dist_travelled();
            if (right) right_pos = right->get_dist_travelled();
            if (middle) middle_pos = middle->get_dist_travelled();
            double delta_left = left_pos - prev_left_pos;
            double delta_right = right_pos - prev_right_pos;
            double delta_middle = middle_pos - prev_middle_pos;

            if (imu) {
                double angle = -to_radians(imu->get_rotation()) * imu_dir;
                dtheta = norm_delta(angle - prev_angle);
                prev_angle += dtheta;
            } else if (left && right) {
                dtheta = (delta_right - delta_left) / (2 * left_right_dist);
                prev_angle += dtheta;
            }

            if (left && right) {
                dx = (delta_left + delta_right) / 2;
            } else if (left) {
                dx = delta_left + dtheta * left_right_dist;
            } else {
                dx = delta_right - dtheta * left_right_dist;
            }
            if (middle) {
                dy = delta_middle + dtheta * middle_dist;
            }

            prev_left_pos = left_pos;
            prev_right_pos = right_pos;
            prev_middle_pos = middle_pos;

            return {dx, dy, dtheta};
        }

        Pose get_velocities() {
            return {};
        }
};

}