#include "VOSS/controller/PIDController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>
#include <utility>

namespace voss::controller {

PIDController::PIDController(std::shared_ptr<localizer::AbstractLocalizer> l)
    : AbstractController(std::move(l)), prev_lin_err(0.0), total_lin_err(0.0),
      prev_ang_err(0.0), total_ang_err(0.0) {
}

chassis::DiffChassisCommand PIDController::get_command(bool reverse,
                                                       bool thru) {
    counter += 10;
    int dir = reverse ? -1 : 1;
    Point current_pos = this->l->get_position();
    double current_angle = this->l->get_orientation_rad();
    bool chainedExecutable = false;
    bool noPose = this->target.theta == 361;

    double dx = target.x - current_pos.x;
    double dy = target.y - current_pos.y;

    double distance_error = sqrt(dx * dx + dy * dy);

    double angle_error = atan2(dy, dx) - current_angle;

    if (reverse) {
        angle_error = atan2(-dy, -dx) - current_angle;
    }

    angle_error = voss::norm_delta(angle_error);

    double lin_speed;

    if (distance_error <= exit_error ||
        (distance_error < min_error && fabs(cos(angle_error)) <= 0.1)) {
        if (thru) {
            chainedExecutable = true;
        } else {
            total_lin_err = 0;
            close += 10;
        }
    } else {
        close = 0;
    }

    if (close > settle_time) {
        return chassis::DiffChassisCommand{chassis::Stop{}};
    }

    if (fabs(distance_error - prev_lin_err) < 0.1 &&
        fabs(current_angle - prev_angle) < voss::to_radians(0.1) &&
        counter > 400) {
        if (thru) {
            chainedExecutable = true;
        }
        close_2 += 10;
    } else {
        close_2 = 0;
    }

    prev_angle = current_angle;

    if (close_2 > settle_time * 2) {
        return chassis::DiffChassisCommand{chassis::Stop{}};
    }

    lin_speed = (thru ? 100.0 : (linear_pid(distance_error))) * dir;

    double ang_speed;
    if (distance_error < min_error) {
        this->can_reverse = true;

        if (noPose) {
            ang_speed = 0; // disable turning when close to the point to prevent
                           // spinning
        } else {
            // turn to face the finale pose angle if executing a pose movement
            double poseError = (target.theta * M_PI / 180) - current_angle;
            while (fabs(poseError) > M_PI)
                poseError -= 2 * M_PI * poseError / fabs(poseError);
            ang_speed = angular_pid(poseError);
        }

        // reduce the linear speed if the bot is tangent to the target
        lin_speed *= cos(angle_error);

    } else {
        if (fabs(angle_error) > M_PI_2 && this->can_reverse) {
            angle_error =
                angle_error - (angle_error / fabs(angle_error)) * M_PI;
            lin_speed = -lin_speed;
        }

        ang_speed = angular_pid(angle_error);
    }
    if (chainedExecutable) {
        return chassis::DiffChassisCommand{chassis::diff_commands::Chained{
            dir * 100 - ang_speed,
            dir * 100.0 + ang_speed}};
    }

    return chassis::DiffChassisCommand{chassis::diff_commands::Voltages{
        lin_speed - ang_speed, lin_speed + ang_speed}};
}

chassis::DiffChassisCommand
PIDController::get_angular_command(bool reverse, bool thru,
                                   voss::AngularDirection direction) {
    counter += 10;
    double current_angle = this->l->get_orientation_rad();
    double target_angle = 0;
    if (this->target.theta == 361) {
        Point current_pos = this->l->get_position();
        double dx = this->target.x - current_pos.x;
        double dy = this->target.y - current_pos.y;
        target_angle = atan2(dy, dx);
    } else {
        target_angle = this->angular_target;
    }
    double angular_error = target_angle - current_angle;

    angular_error = voss::norm_delta(angular_error);

    if (fabs(angular_error) < voss::to_radians(5)) {
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

    if (fabs(angular_error) < angular_exit_error) {
        close += 10;
    } else {
        close = 0;
    }

    if (close > settle_time) {
        return chassis::DiffChassisCommand{chassis::Stop{}};
    }
    if (fabs(angular_error - prev_ang_err) < voss::to_radians(0.1) &&
        counter > 400) {
        close_2 += 10;
    } else {
        close_2 = 0;
    }

    if (close_2 > settle_time * 2) {
        return chassis::DiffChassisCommand{chassis::Stop{}};
    }
    double ang_speed = angular_pid(angular_error);
    return chassis::DiffChassisCommand{
        chassis::diff_commands::Voltages{-ang_speed, ang_speed}};
}

double PIDController::linear_pid(double error) {
    total_lin_err += error;

    double speed = linear_kP * error + linear_kD * (error - prev_lin_err) +
                   linear_kI * total_lin_err;

    this->prev_lin_err = error;

    return speed;
}

double PIDController::angular_pid(double error) {
    total_ang_err += error;

    double speed = angular_kP * error + angular_kD * (error - prev_ang_err) +
                   angular_kI * total_ang_err;

    this->prev_ang_err = error;

    return speed;
}

void PIDController::reset() {
    this->prev_lin_err = 0;
    this->total_lin_err = 0;
    this->prev_ang_err = 0;
    this->total_ang_err = 0;
    this->can_reverse = false;
    this->counter = 0;
    this->turn_overshoot = false;
}

} // namespace voss::controller