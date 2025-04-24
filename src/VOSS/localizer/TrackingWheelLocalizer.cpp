#include "VOSS/localizer/TrackingWheelLocalizer.hpp"

#include "pros/misc.h"
#include "pros/misc.hpp"
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
      prev_left(0.0), prev_right(0.0), prev_middle(0.0),
      prev_pose(AtomicPose{0.0, 0.0, 0.0}),
      offset({offset.x, offset.y, offset.theta.value_or(0.0)}) {
}

static double left_max, right_max, middle_max;
static double left_min, right_min, middle_min;

auto master = pros::Controller(pros::E_CONTROLLER_MASTER);

void TrackingWheelLocalizer::update() {
    double left_pos = left_tracking_wheel->get_raw_position();
    double right_pos =
        right_tracking_wheel ? right_tracking_wheel->get_raw_position() : 0;
    double middle_pos = middle_tracking_wheel->get_raw_position();

    if (left_pos > left_max) {
        left_max = left_pos;
    } else if (left_pos < left_min) {
        left_min = left_pos;
    }
    // if (right_pos > right_max) {
    // right_max = right_pos;
    // } else if (right_pos < right_min) {
    // right_min = right_pos;
    // }
    if (middle_pos > middle_max) {
        middle_max = middle_pos;
    } else if (middle_pos < middle_min) {
        middle_min = middle_pos;
    }

    if (std::abs(left_pos) > 1E6) {
        std::cout << "left_pos above 1E6: " << left_pos << std::endl;
        left_pos = prev_left;
    }
    // if (std::abs(right_pos) > 1E6) {
    // std::cout << "right_pos above 1E6: " << right_pos << std::endl;
    // right_pos = prev_right;
    // }
    if (std::abs(middle_pos) > 1E6) {
        std::cout << "middle_pos above 1E6: " << middle_pos << std::endl;
        middle_pos = prev_middle;
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        std::cout
            << "left max: " << left_max << " left min: " << left_min
            << std::endl
            // << "right max: " << right_max << " right min: " << right_min
            // << std::endl
            << "middle max: " << middle_max << " middle min: " << middle_min
            << std::endl;
    }

    double track_width = 2.0 * left_right_dist;

    double delta_left = 0.0;
    delta_left = (left_pos - prev_left) / left_tracking_wheel->get_tpi();

    if (right_tracking_wheel) {
        double delta_right = 0.0;
        delta_right =
            (right_pos - prev_right) / right_tracking_wheel->get_tpi();
    }

    double delta_middle = 0.0;
    delta_middle =
        (middle_pos - prev_middle) / middle_tracking_wheel->get_tpi();

    double delta_angle = 0.0;
    if (imu.size() > 0) {
        std::vector<double> rotations;
        for (const auto& s_imu : this->imu) {
            if (s_imu->is_installed()) {
                rotations.push_back(s_imu->get_rotation());
            }
        }
        double rotation_avg = voss::get_avg(rotations);
        pose.theta = -to_radians(rotation_avg);
    } else if (right_tracking_wheel) {
        pose.theta = (right_pos / right_tracking_wheel->get_tpi() -
                      left_pos / left_tracking_wheel->get_tpi()) /
                     track_width;
    }
    delta_angle = this->pose.theta - prev_pose.theta;
    // if (std::abs(delta_angle) > 0.16) {
    // delta_angle = 0.0;
    // std::cout << "Delta angle above 0.16\n";
    // } else {
    // }

    prev_left = left_pos;
    // prev_right = right_pos;
    prev_middle = middle_pos;
    prev_pose = pose;

    double local_x;
    double local_y;

    if (delta_angle) {
        double i = sin(delta_angle / 2.0) * 2.0;
        local_x = (delta_left / delta_angle + left_right_dist) * i;
        local_y = (delta_middle / delta_angle + middle_dist) * i;
    } else {
        local_x = delta_left;
        local_y = delta_middle;
    }

    double p = this->pose.theta - delta_angle / 2.0; // global angle

    // convert to absolute displacement
    this->pose.x += cos(p) * local_x - sin(p) * local_y;
    this->pose.y += sin(p) * local_x + cos(p) * local_y;
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
    /*for (const auto &i : this->imu) {
        i->reset(true);
    }*/
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
    this->prev_pose = this->pose;
}

void TrackingWheelLocalizer::set_pose(double x, double y, double theta) {
    this->set_pose({x, y, theta});
}

void TrackingWheelLocalizer::set_horizontal_offset(double horizontal_offset) {
    this->horizontal_offset = horizontal_offset;
}

} // namespace voss::localizer
