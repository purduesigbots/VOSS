#include "VOSS/controller/CRPIDController.hpp"
#include "VOSS/utils/angle.hpp"

namespace voss::controller {
CRPIDController::CRPIDController(std::shared_ptr<localizer::AbstractLocalizer> l)
    : AbstractController(std::move(l)), prev_lin_err(0.0), total_lin_err(0.0),
      prev_ang_err(0.0), total_ang_err(0.0) {
}

chassis::DiffChassisCommand CRPIDController::get_command(bool reverse, bool thru) {
    if (this->target.theta == 361) return chassis::Stop{};
    counter += 10;
    int dir = reverse ? -1 : 1;
    Point current_pos = this->l->get_position();
    double current_angle = this->l->get_orientation_rad();
    double target_angle = voss::to_radians(target.theta);
    bool chainedExecutable = false;

    double dx = target.x - current_pos.x;
    double dy = target.y - current_pos.y;

    double distance_error = sqrt(dx * dx + dy * dy);

    double phi = atan2(dy, dx);

    double alpha = voss::norm_delta(phi - target_angle);
    double beta = atan2(r, distance_error);

    if (alpha < 0) {
        beta = -beta;
    }

    double phi_error;
    if (fabs(alpha) < fabs(beta) || use_alpha) {
        use_alpha = true;
        if (counter % 100 == 0) {
            printf("use alpha\n");
        }
        phi_error = voss::norm_delta(phi - current_angle + alpha);
    } else {
        if (counter % 100 == 0) {
            printf("use beta\n");
        }
        phi_error = voss::norm_delta(phi - current_angle + beta);
    }

    double lin_speed = linear_pid(distance_error);

    if (distance_error < min_error) {
        if (counter % 100 == 0) {
            printf("within min_error\n");
        }
        can_reverse = true;
        lin_speed *= cos(phi_error);
        phi_error = voss::norm_delta(target_angle - current_angle);
    } else if (fabs(phi_error) > M_PI_2 && this->can_reverse) {
        phi_error = voss::norm_delta(phi_error + M_PI);
        lin_speed = -lin_speed;
    }

    if (counter % 100 == 0) {
        printf("x: %f, y: %f\n", current_pos.x, current_pos.y);
        printf("target angle: %f\n", voss::to_degrees(voss::norm(current_angle + phi_error)));
    }

    if (distance_error <= exit_error ||
        (distance_error < min_error && fabs(cos(phi)) <= 0.1)) {
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
        return chassis::Stop{};
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
        return chassis::Stop{};
    }

    double ang_speed = angular_pid(phi_error);

    return chassis::diff_commands::Voltages{lin_speed - ang_speed, lin_speed + ang_speed};
}

chassis::DiffChassisCommand CRPIDController::get_angular_command(bool reverse, bool thru, voss::AngularDirection direction) {
    return chassis::Stop{};
}

double CRPIDController::linear_pid(double error) {
    total_lin_err += error;

    double speed = linear_kP * error + linear_kD * (error - prev_lin_err) +
                   linear_kI * total_lin_err;

    this->prev_lin_err = error;

    return speed;
}

double CRPIDController::angular_pid(double error) {
    total_ang_err += error;

    double speed = angular_kP * error + angular_kD * (error - prev_ang_err) +
                   angular_kI * total_ang_err;

    this->prev_ang_err = error;

    return speed;
}

void CRPIDController::reset() {
    this->prev_lin_err = 0;
    this->total_lin_err = 0;
    this->prev_ang_err = 0;
    this->total_ang_err = 0;
    this->can_reverse = false;
    this->use_alpha = false;
    this->counter = 0;
}
}