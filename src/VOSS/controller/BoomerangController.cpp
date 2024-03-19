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
    double k = this->min_vel / 100;
    Point virtualTarget = {target.x + k * cos(target.theta.value()),
                           target.y + k * sin(target.theta.value())};
    int dir = reverse ? -1 : 1;
    Pose trueTarget;
    double dx = target.x - current_pos.x;
    double dy = target.y - current_pos.y;

    double distance_error = sqrt(dx * dx + dy * dy);
    double at = voss::to_radians(target.theta.value());

    this->carrotPoint = {this->target.x - distance_error * cos(at) * lead_pct,
                         this->target.y - distance_error * sin(at) * lead_pct,
                         target.theta};
    double current_angle =
        this->l->get_orientation_rad() + (reverse ? M_PI : 0);
    bool chainedExecutable = false;
    bool noPose = !this->target.theta.has_value();
    double angle_error;
    angle_error = atan2(dy, dx) - current_angle;

    angle_error = voss::norm_delta(angle_error);

    double lin_speed = linear_pid(distance_error);
    if (thru) {
        lin_speed = copysign(fmax(fabs(lin_speed), this->min_vel), lin_speed);
    }
    lin_speed *= dir;

    double ang_speed;
    if (distance_error < min_error) {
        this->can_reverse = true;

        if (noPose) {
            ang_speed = 0; // disable turning when close to the point to prevent
                           // spinning
        } else {
            // turn to face the finale pose angle if executing a pose movement
            double poseError = target.theta.value() - current_angle;
            while (fabs(poseError) > M_PI)
                poseError -= 2 * M_PI * poseError / fabs(poseError);
            ang_speed = angular_pid(poseError);
        }

        // reduce the linear speed if the bot is tangent to the target

    } else {
        if (fabs(angle_error) > M_PI_2 && this->can_reverse) {
            angle_error =
                angle_error - (angle_error / fabs(angle_error)) * M_PI;
            lin_speed = -lin_speed;
        }

        ang_speed = angular_pid(angle_error);
    }

    lin_speed *= cos(angle_error);

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

double BoomerangController::linear_pid(double error) {
    total_lin_err += error;

    this->vel = error - prev_ang_err;

    double speed = linear_kP * error + linear_kD * (error - prev_lin_err) +
                   linear_kI * total_lin_err;

    this->prev_lin_err = error;

    return speed;
}

double BoomerangController::angular_pid(double error) {
    total_ang_err += error;

    double speed = angular_kP * error + angular_kD * (error - prev_ang_err) +
                   angular_kI * total_ang_err;

    this->prev_ang_err = error;

    return speed;
}

void BoomerangController::reset() {
    this->prev_lin_err = 0;
    this->total_lin_err = 0;
    this->prev_ang_err = 0;
    this->total_ang_err = 0;
    this->can_reverse = false;
    this->counter = 0;
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
BoomerangController::modify_exit_error(double exit_error) {
    auto pid_mod = BoomerangControllerBuilder::from(*this)
                       .with_exit_error(exit_error)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<BoomerangController>
BoomerangController::modify_angular_exit_error(double exit_error) {
    auto pid_mod = BoomerangControllerBuilder::from(*this)
                       .with_angular_exit_error(exit_error)
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

std::shared_ptr<BoomerangController>
BoomerangController::modify_settle_time(double settle_time) {
    auto pid_mod = BoomerangControllerBuilder::from(*this)
                       .with_settle_time(settle_time)
                       .build();

    this->p = pid_mod;

    return this->p;
}

} // namespace voss::controller