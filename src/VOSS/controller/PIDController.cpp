#include "VOSS/controller/PIDController.hpp"
#include "PIDControllerBuilder.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/utils/angle.hpp"
#include "VOSS/utils/debug.hpp"
#include <cmath>
#include <memory>
#include <utility>

namespace voss::controller {

PIDController::PIDController(std::shared_ptr<localizer::AbstractLocalizer> l)
    : AbstractController(std::move(l)) {
}

chassis::DiffChassisCommand
PIDController::get_command(bool reverse, bool thru,
                           std::shared_ptr<AbstractExitCondition> ec) {
    // Runs in background of move commands
    // distance formula to find the distance between the robot and the target
    // position distance error is distance ArcTan is used to find the angle
    // between the robot and the target position This difference is the angle
    // error Power applied to the motors based on type of movement and error
    // Dependant on the weight of the constants and the errors
    // Controller exits when robot settled for a designated time duration or
    // accurate to a specified tolerance
    int dir = reverse ? -1 : 1;
    Point current_pos = this->l->get_position();
    double current_angle = this->l->get_orientation_rad();
    bool noPose = !this->target.theta.has_value();

    double dx = target.x - current_pos.x;
    double dy = target.y - current_pos.y;

    double distance_error = sqrt(dx * dx + dy * dy);

    double angle_error = atan2(dy, dx) - current_angle;

    if (reverse) {
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

        if (noPose) {
            switch (min_err_behavior) {
                case MinErrBehavior::NO_ANG_PID:
                    ang_speed = 0;
                    break;
                case MinErrBehavior::SCALE_ANG_PID: {
                    double min_err_2 = min_error / 2;
                    double scale_factor = (distance_error - min_err_2) / min_err_2;
                    scale_factor = fmax(0.0, scale_factor);
                    ang_speed = scale_factor * angular_pid.update(angle_error);
                    break;
                }
                case MinErrBehavior::MAINTAIN_HEADING: {
                    if (std::isnan(min_err_heading)) {
                        min_err_heading = current_angle;
                    }
                    double min_err_heading_error = norm_delta(min_err_heading - current_angle);
                    ang_speed = angular_pid.update(min_err_heading_error);
                    break;
                }
            }
        } else {
            // turn to face the finale pose angle if executing a pose movement
            double poseError = target.theta.value() - current_angle;

            while (fabs(poseError) > M_PI)
                poseError -= 2 * M_PI * poseError / fabs(poseError);
            ang_speed = angular_pid.update(poseError);
        }

    } else {
        min_err_heading = NAN;
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
        printf("dist_err: %.2f, ang_err: %.2f, lin_speed: %.2f, ang_speed: %.2f\n", distance_error, angle_error, lin_speed, ang_speed);
    }

    // Runs at the end of a through movement
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

chassis::DiffChassisCommand
PIDController::get_angular_command(bool reverse, bool thru,
                                   voss::AngularDirection direction,
                                   std::shared_ptr<AbstractExitCondition> ec) {
    // Runs in the background of turn commands
    // Error is the difference between the target angle and the current angle
    // ArcTan is used to find the angle between the robot and the target
    // position Output is proportional to the error and the weight of the
    // constant Controller exits when robot settled for a designated time
    // duration or accurate to a specified tolerance
    double current_angle = this->l->get_orientation_rad();
    double target_angle = 0;
    if (!this->target.theta.has_value()) {
        Point current_pos = this->l->get_position();
        double dx = this->target.x - current_pos.x;
        double dy = this->target.y - current_pos.y;
        target_angle = atan2(dy, dx);
    } else {
        target_angle = this->angular_target;
    }
    double angular_error = target_angle - current_angle;
    bool chainedExecutable = false;
    angular_error = voss::norm_delta(angular_error);

    if (fabs(angular_error) < voss::to_radians(5)) {
        if (thru) {
            chainedExecutable = true;
        }
        turn_overshoot = true;
    }

    if (!turn_overshoot) {
        if (direction == voss::AngularDirection::COUNTERCLOCKWISE &&
            angular_error < 0) {
            angular_error += 2 * M_PI;
        } else if (direction == voss::AngularDirection::CLOCKWISE &&
                   angular_error > 0) {
            angular_error -= 2 * M_PI;
        }
    }

    double ang_speed = angular_pid.update(angular_error);
    if (ec->is_met(this->l->get_pose(), thru) || chainedExecutable) {
        if (thru) {
            return chassis::DiffChassisCommand{
                chassis::diff_commands::Chained{-ang_speed, ang_speed}};
        }
        return chassis::Stop{};
    }
    return chassis::DiffChassisCommand{
        chassis::diff_commands::Voltages{-ang_speed, ang_speed}};
}

// Function to reset every variable used in the PID controller
void PIDController::reset() {
    this->linear_pid.reset();
    this->angular_pid.reset();
    this->can_reverse = false;
    this->turn_overshoot = false;
    min_err_heading = NAN;
}

std::shared_ptr<PIDController>
PIDController::modify_linear_constants(double kP, double kI, double kD) {
    auto pid_mod = PIDControllerBuilder::from(*this)
                       .with_linear_constants(kP, kI, kD)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<PIDController>
PIDController::modify_angular_constants(double kP, double kI, double kD) {
    auto pid_mod = PIDControllerBuilder::from(*this)
                       .with_angular_constants(kP, kI, kD)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<PIDController>
PIDController::modify_min_error(double min_error) {
    auto pid_mod =
        PIDControllerBuilder::from(*this).with_min_error(min_error).build();

    this->p = pid_mod;

    return this->p;
}

void PIDController::set_thru_behavior(ThruBehavior thru_behavior) {
    this->thru_behavior = thru_behavior;
}

void PIDController::set_cosine_scaling(CosineScaling cosine_scaling) {
    this->cosine_scaling = cosine_scaling;
}

void PIDController::set_min_err_behavior(MinErrBehavior min_err_behavior) {
    this->min_err_behavior = min_err_behavior;
}

void PIDController::set_overturn(bool overturn) {
    enable_overturn = overturn;
}

} // namespace voss::controller