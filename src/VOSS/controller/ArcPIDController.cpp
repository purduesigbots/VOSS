#include "VOSS/controller/ArcPIDController.hpp"

#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>
#include <memory>

namespace voss::controller {

ArcPIDController::ArcPIDController(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : AbstractController(l), prev_lin_err(0.0), total_lin_err(0.0) {
}

chassis::ChassisCommand
ArcPIDController::get_command(bool reverse, bool thru,
                              std::shared_ptr<AbstractExitCondition> ec) {
    Point current_pos = this->l->get_position();
    double current_angle = this->l->get_orientation_rad();

    // x: current_x, y: current_y
    // x': target_x, y': target_y
    // sin: sin(current_angle), cos: cos(current_angle)
    // line 1: (x, y) + t(-sin, cos)
    // line 2: ((x+x')/2, (y+y')/2) + s(-(y'-y), (x'-x))
    // equation 1: t(-sin) + s(y'-y) = (x'-x)/2
    // equation 2: t(cos) + s(x-x') = (y'-y)/2
    double a = -sin(current_angle);
    double b = this->target.y - current_pos.y;
    double c = (this->target.x - current_pos.x) / 2;
    double d = cos(current_angle);
    double e = current_pos.x - this->target.x;
    double f = (this->target.y - current_pos.y) / 2;

    // apply cramer's rule to solve for t
    double t;
    if (fabs(a * e - d * b) < 0.00001) {
        t = 0.0;
    } else {
        t = (c * e - f * b) / (a * e - d * b);
    }

    double distance_error = sqrt(b * b + e * e);
    double angle_error = atan2(b, -e) - current_angle;

    if (reverse) {
        angle_error = atan2(-b, -e) - current_angle;
    }

    angle_error = voss::norm_delta(angle_error);

    if (distance_error <= exit_error ||
        (distance_error < min_error && fabs(cos(angle_error)) <= 0.1)) {
        this->close += 10;
    } else {
        this->close = 0;
    }

    double lin_speed = thru ? 100.0 : this->linear_pid(distance_error);

    if (distance_error < this->min_error) {
        this->can_reverse = true;
        lin_speed *= cos(angle_error);
        // return chassis::ChassisCommand{chassis::Voltages{lin_speed,
        // lin_speed}};
        t = prev_t;
    } else if (fabs(angle_error) > M_PI_2 && this->can_reverse) {
        lin_speed = -lin_speed;
    }

    if (lin_speed > prev_lin_speed + slew) {
        lin_speed = prev_lin_speed + slew;
    }
    lin_speed *= reverse ? -1 : 1;

    double left_speed, right_speed;
    if (t != 0.0) {
        // left_speed = (t - track_width / 2) / t * lin_speed;
        // right_speed = (t + track_width / 2) / t * lin_speed;
        left_speed = lin_speed * (2 - track_width / t) / 2;
        right_speed = lin_speed * (2 + track_width / t) / 2;
    } else {
        left_speed = lin_speed;
        right_speed = lin_speed;
    }
    prev_t = t;
    prev_lin_speed = lin_speed;

    if (ec->is_met(this->l->get_pose())) {
        return chassis::ChassisCommand{chassis::Stop{}};
    }

    return chassis::ChassisCommand{chassis::Voltages{left_speed, right_speed}};
}

chassis::ChassisCommand ArcPIDController::get_angular_command(
    bool reverse, bool thru, std::shared_ptr<AbstractExitCondition> ec) {
    return chassis::ChassisCommand{chassis::Stop{}};
}

double ArcPIDController::linear_pid(double error) {
    this->total_lin_err += error;

    double speed = linear_kP * error + linear_kD * (error - prev_lin_err) +
                   linear_kI * total_lin_err;

    this->prev_lin_err = error;

    return speed;
}

void ArcPIDController::reset() {
    this->prev_lin_err = 0.0;
    this->total_lin_err = 0.0;
    this->prev_lin_speed = 0.0;
}

} // namespace voss::controller