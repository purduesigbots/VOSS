#include "VOSS/controller/BoomerangController.hpp"
#include "BoomerangControllerBuilder.hpp"
#include "PIDController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/utils/angle.hpp"
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
    Pose trueTarget;
    double dx = target.x - current_pos.x;
    double dy = target.y - current_pos.y;

    double distance_error = sqrt(dx * dx + dy * dy);
    double at = target.theta.value();

    this->carrotPoint = {this->target.x - distance_error * cos(at) * lead_pct,
                         this->target.y - distance_error * sin(at) * lead_pct,
                         target.theta};
    dx = carrotPoint.x - current_pos.x;
    dy = carrotPoint.y - current_pos.y;
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
        lin_speed = copysign(fmax(fabs(lin_speed), this->min_vel), lin_speed);
    }
    lin_speed *= dir;

    double pose_error = voss::norm_delta(target.theta.value() - current_angle);

    double ang_speed;
    if (distance_error < min_error) {
        this->can_reverse = true;

        ang_speed = angular_pid.update(pose_error);
    } else if (distance_error < 2 * min_error) {
        double scale_factor = (distance_error - min_error) / min_error;
        double scaled_angle_error = voss::norm_delta(
            scale_factor * angle_error + (1 - scale_factor) * pose_error);

        ang_speed = angular_pid.update(scaled_angle_error);
    } else {
        if (fabs(angle_error) > M_PI_2 && this->can_reverse) {
            angle_error =
                angle_error - (angle_error / fabs(angle_error)) * M_PI;
            lin_speed = -lin_speed;
        }

        ang_speed = angular_pid.update(angle_error);
    }

    lin_speed *= cos(angle_error);

    lin_speed = std::max(-100.0, std::min(100.0, lin_speed));

    if (ec->is_met(this->l->get_pose(), thru)) {
        if (thru) {
            return chassis::DiffChassisCommand{chassis::diff_commands::Chained{
                dir * std::fmax(lin_speed, this->min_vel) - ang_speed,
                dir * std::fmax(lin_speed, this->min_vel) + ang_speed}};
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

} // namespace voss::controller