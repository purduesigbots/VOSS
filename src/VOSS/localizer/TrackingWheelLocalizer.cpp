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
    std::vector<std::unique_ptr<pros::IMU>> imu, double left_distance,
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



    double prev_theta = this->pose.theta;
    this->pose.theta += delta_theta;
    double mid_theta = prev_theta + delta_theta / 2.0;
    // update current pose
    this->pose.x += delta_x_ref * cos(mid_theta) - delta_y_ref * sin(mid_theta);
    this->pose.y += delta_y_ref * cos(mid_theta) + delta_x_ref * sin(mid_theta);


    //    double track_width = 2.0 * left_right_dist;
    //
    //    double delta_left = 0.0;
    //    delta_left = (left_pos - prev_left) / left_tracking_wheel->get_tpi();
    //
    //    double delta_right = 0.0;
    //    delta_right = (right_pos - prev_right) /
    //    right_tracking_wheel->get_tpi();
    //
    //    double delta_middle = 0.0;
    //    delta_middle =
    //        (middle_pos - prev_middle) / middle_tracking_wheel->get_tpi();
    //
    //    double delta_angle = 0.0;
    //    delta_angle = (delta_right - delta_left) / track_width;
    //    this->pose.theta += delta_angle;
    //
    //    prev_left = left_pos;
    //    prev_right = right_pos;
    //    prev_middle = middle_pos;
    //    prev_pose = pose;
    //
    //    double local_x;
    //    double local_y;
    //
    //    if (delta_angle) {
    //        double i = sin(delta_angle / 2.0) * 2.0;
    //        local_x = (delta_right / delta_angle - left_right_dist) * i;
    //        local_y = (delta_middle / delta_angle + middle_dist) * i;
    //    } else {
    //        local_x = delta_right;
    //        local_y = delta_middle;
    //    }
    //
    //    double p = this->pose.theta - delta_angle / 2.0; // global angle
    //
    //    // convert to absolute displacement
    //    this->pose.x += cos(p) * local_x - sin(p) * local_y;
    //    this->pose.y += sin(p) * local_x + cos(p) * local_y;
    /*double left = left_tracking_wheel->get_raw_position();
    double right = right_tracking_wheel->get_raw_position();
    double middle = middle_tracking_wheel->get_raw_position();

    double delta_left = (left - prev_left) / left_tracking_wheel->get_tpi();
    double delta_right = (right - prev_right) / right_tracking_wheel->get_tpi();
    double delta_middle =
        (middle - prev_middle) / middle_tracking_wheel->get_tpi();

    prev_left = left;
    prev_right = right;
    prev_middle = middle;

    double delta_theta = (delta_right - delta_left) / (2.0 * left_right_dist);

    double delta_local_x;
    double delta_local_y;

    if (delta_theta == 0.0) {
        delta_local_x = delta_middle;
        delta_local_y = delta_right;
    } else {
        double i = 2.0 * std::sin(delta_theta / 2.0);
        delta_local_x = i * (delta_middle / delta_theta + middle_dist);
        delta_local_y = i * (delta_right / delta_theta + left_right_dist);
    }

    double theta_m = this->prev_pose.theta + delta_theta / 2.0;

    double pl_r = std::hypot(delta_local_x, delta_local_y);
    double pl_theta = std::atan2(delta_local_y, delta_local_x);
    pl_theta -= theta_m;

    double delta_x = pl_r * std::cos(pl_theta);
    double delta_y = pl_r * std::sin(pl_theta);

    this->pose.x = this->prev_pose.x + delta_x;
    this->pose.y = this->prev_pose.y + delta_y;
    this->pose.theta = this->prev_pose.theta + delta_theta;

    this->prev_pose.x = this->pose.x.load();
    this->prev_pose.y = this->pose.y.load();
    this->prev_pose.theta = this->pose.theta.load();*/

    /*
        double delta_left = 0.0;
        double delta_right = 0.0;
        double delta_middle = 0.0;
        double delta_angle = 0.0;

        if (left_tracking_wheel) {
            delta_left = left_tracking_wheel->get_dist_travelled() - prev_left;
        }
        if (right_tracking_wheel) {
            delta_right = right_tracking_wheel->get_dist_travelled() -
       prev_right;
        }
        if (middle_tracking_wheel) {
            delta_middle =
                middle_tracking_wheel->get_dist_travelled() - prev_middle;
        }
        delta_angle = (delta_right - delta_left) / (2 * left_right_dist);
        pose.theta += delta_angle;

        prev_left += delta_left;
        prev_right += delta_right;
        prev_middle += delta_middle;
        prev_pose = pose;

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

        double p = this->pose.theta - delta_angle / 2.0; // global angle

        // convert to absolute displacement
        this->pose.x += cos(p) * local_x - sin(p) * local_y;
        this->pose.y += sin(p) * local_x + cos(p) * local_y;*/
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
}

void TrackingWheelLocalizer::set_pose(double x, double y, double theta) {
    this->set_pose({x, y, theta});
}

void TrackingWheelLocalizer::set_horizontal_offset(double horizontal_offset) {
    this->horizontal_offset = horizontal_offset;
}

} // namespace voss::localizer
