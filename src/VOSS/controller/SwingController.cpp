#include "VOSS/controller/SwingController.hpp"

#include <utility>
#include "VOSS/utils/angle.hpp"

namespace voss::controller {
SwingController::SwingController(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : AbstractController(std::move(l)){};

chassis::DiffChassisCommand SwingController::get_command(bool reverse,
                                                         bool thru) {
    return chassis::DiffChassisCommand{chassis::Stop{}};
}

chassis::DiffChassisCommand
SwingController::get_angular_command(bool reverse, bool thru,
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
    chassis::DiffChassisCommand command;
    if (!((ang_speed >= 0.0) ^ (this->prev_ang_speed < 0.0)) &&
        this->prev_ang_speed != 0) {
        can_reverse = !can_reverse;
    }

    if (!this->can_reverse) {
        if (!reverse) {
            command = std::signbit(ang_speed)
                          ? chassis::diff_commands::Swing{-ang_speed, 0}
                          : chassis::diff_commands::Swing{0, ang_speed};
        } else {
            command = std::signbit(ang_speed)
                          ? chassis::diff_commands::Swing{0, ang_speed}
                          : chassis::diff_commands::Swing{-ang_speed, 0};
        }
    } else {
        if (reverse) {
            command = std::signbit(ang_speed)
                          ? chassis::diff_commands::Swing{-ang_speed, 0}
                          : chassis::diff_commands::Swing{0, ang_speed};
        } else {
            command = std::signbit(ang_speed)
                          ? chassis::diff_commands::Swing{0, ang_speed}
                          : chassis::diff_commands::Swing{-ang_speed, 0};
        }
    }

    prev_ang_speed = ang_speed;

    return command;
}

double SwingController::angular_pid(double error) {
    total_ang_err += error;

    double speed = angular_kP * error + angular_kD * (error - prev_ang_err) +
                   angular_kI * total_ang_err;

    this->prev_ang_err = error;

    return speed;
}

void SwingController::reset() {
    this->prev_ang_err = 0;
    this->prev_ang_speed = 0;
    this->total_ang_err = 0;
    this->counter = 0;
    this->can_reverse = false;
    this->turn_overshoot = false;
}
}; // namespace voss::controller
