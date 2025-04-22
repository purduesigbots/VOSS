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
    std::shared_ptr<pros::IMU> imu, double left_distance,
    double right_distance, double middle_dist, Pose offset)
    : AbstractLocalizer(), left_tracking_wheel(std::move(left)),
      right_tracking_wheel(std::move(right)),
      middle_tracking_wheel(std::move(middle)), imu(std::move(imu)),
      left_distance(left_distance), right_distance(right_distance),
      middle_dist(middle_dist), prev_left(0.0), prev_right(0.0),
      prev_middle(0.0),
      offset({offset.x, offset.y, offset.theta.value_or(0.0)}) {
}

void TrackingWheelLocalizer::update() {

    /*
    delta_fwd = (R * left_distance - L * right_distance) / (left_distance -
    right_distance)
     delta_str: delta_strafe = M - middle_distance * delta_theta

    delta_theta

     theta = (R - L) / (left_distance - right_distance)

     r_0 = delta_fwd / delta_theta
     r_1 = delta_str / delta_theta

     delta_x_ref = r_0 * sin(delta_theta) - r_1 * (1.0 - cos(delta_theta))
     delta_y_ref = r_1 * sin(delta_theta) + r_0 * (1.0 - cos(delta_theta))

     cur_x = prev_x + delta_x_ref * cos(cur_theta) - delta_y_ref *
    sin(cur_theta)
     cur_y = prev_y + delta_y_ref * cos(cur_theta) + delta_x_ref *
    sin(cur_theta)
     */

    double left_pos = left_tracking_wheel->get_raw_position();
    double right_pos = right_tracking_wheel->get_raw_position();
    double middle_pos = middle_tracking_wheel->get_raw_position();

    double delta_left =
        left_tracking_wheel->get_dist_travelled(left_pos - prev_left);
    double delta_right =
        right_tracking_wheel->get_dist_travelled(right_pos - prev_right);
    double delta_middle =
        middle_tracking_wheel->get_dist_travelled(middle_pos - prev_middle);

    double imu_heading = voss::to_radians(imu->get_heading());//[0, 2pi)

    prev_left = left_pos;
    prev_right = right_pos;
    prev_middle = middle_pos;

    double delta_fwd =
        (delta_right * left_distance - delta_left * right_distance) /
        (left_distance - right_distance);

    double delta_theta =
        (delta_right - delta_left) / (left_distance - right_distance);

    //    double delta_theta = cur_theta - this->pose.theta;
    double delta_str = delta_middle - middle_dist * delta_theta;

    double delta_x_ref, delta_y_ref;
    static constexpr double epsilon = 1e-6;
    if (std::abs(delta_theta) > epsilon) {
        double r_0 = delta_fwd / delta_theta;
        double r_1 = delta_str / delta_theta;

        double sin_theta = sin(delta_theta);
        double cos_theta = cos(delta_theta);


        delta_x_ref = r_0 * sin_theta - r_1 * (1.0 - cos_theta);
        delta_y_ref = r_1 * sin_theta + r_0 * (1.0 - cos_theta);
    } else {
        delta_x_ref = delta_fwd;
        delta_y_ref = delta_str;
    }

    double mid_theta = this->pose.theta + delta_theta / 2.0;
    Pose x_pred = {
        this->pose.x + delta_x_ref * cos(mid_theta) - delta_y_ref * sin(mid_theta),
        this->pose.y + delta_y_ref * cos(mid_theta) + delta_x_ref * sin(mid_theta),
        this->pose.theta + delta_theta
    };


    if(kalman_filter) {
        Eigen::Vector3d odom_pred_vec = {x_pred.x, x_pred.y, x_pred.theta.value_or(0.0)};
        kalman_filter->step(odom_pred_vec, delta_fwd, delta_str, delta_theta, imu_heading);
        auto pred_pose = kalman_filter->get_predicted_pose();
        this->pose = pred_pose;
    } else {
        this->pose = x_pred;
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

    if(kalman_filter) {
        kalman_filter->set_pose({this->pose.x, this->pose.y, this->pose.theta});
    }
}

void TrackingWheelLocalizer::set_pose(double x, double y, double theta) {
    this->set_pose({x, y, theta});
}

void TrackingWheelLocalizer::set_horizontal_offset(double horizontal_offset) {
    this->horizontal_offset = horizontal_offset;
}

} // namespace voss::localizer
