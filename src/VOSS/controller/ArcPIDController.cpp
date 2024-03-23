#include "VOSS/controller/ArcPIDController.hpp"
#include "ArcPIDControllerBuilder.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>
#include <utility>

namespace voss::controller {

ArcPIDController::ArcPIDController(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : AbstractController(std::move(l)) {
}

chassis::DiffChassisCommand
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

    if (std::isnan(arc_radius)) {
        arc_radius = t;
        arc_center = {
            current_pos.x + t * a,
            current_pos.y + t * d
        };
        //printf("arc center: %f %f\n", arc_center.x, arc_center.y);
    }

    double distance_error = sqrt(b * b + e * e);
    double angle_error = atan2(b, -e) - current_angle;

    if (reverse) {
        angle_error = atan2(-b, e) - current_angle;
    }

    angle_error = voss::norm_delta(angle_error);

    double lin_speed = thru ? 100.0 : this->linear_pid.update(distance_error);

    if (distance_error < this->min_error) {
        this->can_reverse = true;
        lin_speed *= cos(angle_error);
        // return
        // chassis::DiffChassisCommand{chassis::diff_commands::Voltages{lin_speed,
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
    if (arc_radius != 0.0) {
        // left_speed = (t - track_width / 2) / t * lin_speed;
        // right_speed = (t + track_width / 2) / t * lin_speed;
        left_speed = lin_speed * (2 - track_width / arc_radius) / 2;
        right_speed = lin_speed * (2 + track_width / arc_radius) / 2;
        double tangent = atan2(current_pos.y - arc_center.y, current_pos.x - arc_center.x);
        if ((bool)(arc_radius > 0) != reverse) {
            tangent += M_PI_2;
        } else {
            tangent -= M_PI_2;
        }
        //printf("current %f target %f\n", voss::to_degrees(current_angle), voss::to_degrees(tangent));
        printf("x %f y %f\n", current_pos.x, current_pos.y);
        double ang_speed = angular_pid.update(voss::norm_delta(tangent - current_angle));
        left_speed -= ang_speed;
        right_speed += ang_speed;
    } else {
        left_speed = lin_speed;
        right_speed = lin_speed;
    }
    prev_t = t;
    prev_lin_speed = lin_speed;
    if (ec->is_met(this->l->get_pose(), thru)) {
        if (thru) {
            return chassis::DiffChassisCommand{
                chassis::diff_commands::Chained{left_speed, right_speed}};
        } else {
            return chassis::Stop{};
        }
    }
    return chassis::DiffChassisCommand{
        chassis::diff_commands::Voltages{left_speed, right_speed}};
}

chassis::DiffChassisCommand ArcPIDController::get_angular_command(
    bool reverse, bool thru, voss::AngularDirection direction,
    std::shared_ptr<AbstractExitCondition> ec) {
    return chassis::DiffChassisCommand{chassis::Stop{}};
}

void ArcPIDController::reset() {
    this->linear_pid.reset();
    this->angular_pid.reset();
    this->prev_lin_speed = 0.0;
    this->arc_radius = NAN;
}

std::shared_ptr<ArcPIDController>
ArcPIDController::modify_linear_constants(double kP, double kI, double kD) {
    auto pid_mod = ArcPIDControllerBuilder::from(*this)
                       .with_linear_constants(kP, kI, kD)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<ArcPIDController>
ArcPIDController::modify_track_width(double track_width) {
    auto pid_mod = ArcPIDControllerBuilder::from(*this)
                       .with_track_width(track_width)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<ArcPIDController>
ArcPIDController::modify_min_error(double min_error) {
    auto pid_mod =
        ArcPIDControllerBuilder::from(*this).with_min_error(min_error).build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<ArcPIDController> ArcPIDController::modify_slew(double slew) {
    auto pid_mod = ArcPIDControllerBuilder::from(*this).with_slew(slew).build();

    this->p = pid_mod;

    return this->p;
}

} // namespace voss::controller