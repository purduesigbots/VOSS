#include "VOSS/controller/BoomerangController.hpp"
#include "BoomerangControllerBuilder.hpp"
#include "PIDController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/utils/angle.hpp"
#include "VOSS/utils/debug.hpp"
#include <cmath>

namespace voss::controller {

BoomerangController::BoomerangController(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : AbstractController(l) {
}

chassis::DiffChassisCommand
BoomerangController::get_command(bool reverse, bool thru,
                                 std::shared_ptr<AbstractExitCondition> ec) {

    if (!target.theta.has_value()) {
        return chassis::DiffChassisCommand{chassis::Stop{}};
    }

    Point current_pos = this->l->get_position();

    int dir = reverse ? -1 : 1;
    double dx = target.x - current_pos.x;
    double dy = target.y - current_pos.y;

    double distance_error = sqrt(dx * dx + dy * dy);
    double target_angle = target.theta.value();

    Point carrot_point;
    if (!reverse) {
        carrot_point = {target.x - distance_error * cos(target_angle) * lead_pct,
                        target.y - distance_error * sin(target_angle) * lead_pct};
    } else {
        carrot_point = {target.x + distance_error * cos(target_angle) * lead_pct,
                        target.y + distance_error * sin(target_angle) * lead_pct};
    }

    dx = carrot_point.x - current_pos.x;
    dy = carrot_point.y - current_pos.y;
    double current_angle = this->l->get_orientation_rad();

    double angle_error;
    if (!reverse) {
        angle_error = atan2(dy, dx) - current_angle;
    } else {
        angle_error = atan2(-dy, -dx) - current_angle;
    }

    angle_error = voss::norm_delta(angle_error);

    double lin_speed = linear_pid.update(distance_error);
    if (thru) {
        if (thru_behavior == ThruBehavior::FULL_SPEED) {
            lin_speed = 100;
        } else {
            lin_speed = fmax(lin_speed, min_vel);
        }
    }
    if (cosine_scaling == CosineScaling::ALL_THE_TIME || distance_error < min_error) {
        lin_speed *= cos(angle_error);
    }
    lin_speed *= dir;

    double ang_speed;
    if (distance_error < min_error) {
        this->can_reverse = true;
        double pose_error = voss::norm_delta(target_angle - current_angle);

        if (min_err_behavior == MinErrBehavior::TARGET_HEADING) {
            ang_speed = angular_pid.update(pose_error);
        } else {
            double min_err_2 = min_error / 2;
            double scale_factor = (distance_error - min_err_2) / min_err_2;
            scale_factor = fmax(0.0, scale_factor);
            double scaled_angle_error = norm_delta(scale_factor * angle_error + (1 - scale_factor) * pose_error);
            ang_speed = angular_pid.update(scaled_angle_error);
        }
    } else {
        if (fabs(angle_error) > M_PI_2 && this->can_reverse) {
            angle_error =
                angle_error - (angle_error / fabs(angle_error)) * M_PI;
            if (cosine_scaling != CosineScaling::ALL_THE_TIME) {
                lin_speed = -lin_speed;
            }
        }

        ang_speed = angular_pid.update(angle_error);
    }

    lin_speed = std::clamp(lin_speed, -100.0, 100.0);
    if (enable_overturn) {
        ang_speed = std::clamp(ang_speed, -100.0, 100.0);
        lin_speed = std::clamp(lin_speed, -100 + fabs(ang_speed), 100 - fabs(ang_speed));
    }

    if (get_debug()) {
        printf("carrot point: %.2f, %.2f, dist_err: %.2f, ang_err: %.2f, lin_speed: %.2f, ang_speed: %.2f\n", carrot_point.x, carrot_point.y, angle_error, lin_speed, ang_speed);
    }

    if (ec->is_met(this->l->get_pose(), thru)) {
        if (thru) {
            return chassis::DiffChassisCommand{chassis::diff_commands::Chained{
                lin_speed - ang_speed, lin_speed + ang_speed}};
        } else {
            return chassis::Stop{};
        }
    }

    return chassis::DiffChassisCommand{chassis::diff_commands::Voltages{
        lin_speed - ang_speed, lin_speed + ang_speed}};
}

chassis::DiffChassisCommand BoomerangController::get_angular_command(
    bool reverse, bool thru, voss::AngularDirection direction,
    std::shared_ptr<AbstractExitCondition> ec) {
    return chassis::DiffChassisCommand{chassis::Stop{}};
}

void BoomerangController::reset() {
    this->linear_pid.reset();
    this->angular_pid.reset();
    this->can_reverse = false;
}

std::shared_ptr<BoomerangController>
BoomerangController::modify_linear_constants(double kP, double kI, double kD) {
    auto pid_mod = BoomerangControllerBuilder::from(*this)
                       .with_linear_constants(kP, kI, kD)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<BoomerangController>
BoomerangController::modify_angular_constants(double kP, double kI, double kD) {
    auto pid_mod = BoomerangControllerBuilder::from(*this)
                       .with_angular_constants(kP, kI, kD)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<BoomerangController>
BoomerangController::modify_min_error(double min_error) {
    auto pid_mod = BoomerangControllerBuilder::from(*this)
                       .with_min_error(min_error)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<BoomerangController>
BoomerangController::modify_lead_pct(double lead_pct) {
    auto pid_mod =
        BoomerangControllerBuilder::from(*this).with_lead_pct(lead_pct).build();

    this->p = pid_mod;

    return this->p;
}

void BoomerangController::set_thru_behavior(ThruBehavior thru_behavior) {
    this->thru_behavior = thru_behavior;
}

void BoomerangController::set_cosine_scaling(CosineScaling cosine_scaling) {
    this->cosine_scaling = cosine_scaling;
}

void BoomerangController::set_min_err_behavior(MinErrBehavior min_err_behavior) {
    this->min_err_behavior = min_err_behavior;
}

void BoomerangController::set_overturn(bool overturn) {
    enable_overturn = overturn;
}

} // namespace voss::controller