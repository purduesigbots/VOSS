#include "VOSS/controller/SwingController.hpp"

#include "SwingControllerBuilder.hpp"
#include "VOSS/utils/angle.hpp"
#include <utility>

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
    // Runs in background of Swing Turn commands
    // ArcTan is used to find the angle between the robot and the target
    // position One size of the drive is locked in place while the other side
    // moves. This is determined by which side has less required movement
    // Reverse flag flips the direction the moving side
    // Power appled to motors based on proportion of the error and the weight of
    // the constant Exit conditions are when the robot has settled for a
    // designated time duration or accurate to a specified tolerance
    counter += 10;
    bool chainedExecutable = false;
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
        if (thru) {
            chainedExecutable = true;
        }
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
    if (thru) {
        ang_speed = copysign(fmax(fabs(ang_speed), this->min_vel), ang_speed);
    }

    if (!((ang_speed >= 0.0) ^ (this->prev_ang_speed < 0.0)) &&
        this->prev_ang_speed != 0) {
        can_reverse = !can_reverse;
    }
    //    if (chainedExecutable) {
    //        if (!this->can_reverse) {
    //            if (!reverse) {
    //                return std::signbit(ang_speed)
    //                              ?
    //                              chassis::diff_commands::Chained{-ang_speed,
    //                              0} : chassis::diff_commands::Chained{0,
    //                              ang_speed};
    //            }
    //            return std::signbit(ang_speed)
    //                       ? chassis::diff_commands::Chained{0, ang_speed}
    //                       : chassis::diff_commands::Chained{-ang_speed, 0};
    //        } else {
    //            if (reverse) {
    //                return std::signbit(ang_speed)
    //                              ?
    //                              chassis::diff_commands::Chained{-ang_speed,
    //                              0} : chassis::diff_commands::Chained{0,
    //                              ang_speed};
    //            }
    //            return std::signbit(ang_speed)
    //                       ? chassis::diff_commands::Chained{0, ang_speed}
    //                       : chassis::diff_commands::Chained{-ang_speed, 0};
    //        }
    //    } else {
    //        if (!this->can_reverse) {
    //            if (!reverse) {
    //                return std::signbit(ang_speed)
    //                           ? chassis::diff_commands::Swing{-ang_speed, 0}
    //                           : chassis::diff_commands::Swing{0, ang_speed};
    //            }
    //            return std::signbit(ang_speed)
    //                       ? chassis::diff_commands::Swing{0, ang_speed}
    //                       : chassis::diff_commands::Swing{-ang_speed, 0};
    //        } else {
    //            if (reverse) {
    //                return std::signbit(ang_speed)
    //                           ? chassis::diff_commands::Swing{-ang_speed, 0}
    //                           : chassis::diff_commands::Swing{0, ang_speed};
    //            }
    //            return std::signbit(ang_speed)
    //                       ? chassis::diff_commands::Swing{0, ang_speed}
    //                       : chassis::diff_commands::Swing{-ang_speed, 0};
    //        }
    //    };

    prev_ang_speed = ang_speed;
    if (chainedExecutable) {
        if (this->can_reverse ^ reverse) { // same sign
            return std::signbit(ang_speed)//1
                       ? chassis::diff_commands::Chained{-ang_speed, 0}
                       : chassis::diff_commands::Chained{0, ang_speed};
        }
        return std::signbit(ang_speed)//2
                   ? chassis::diff_commands::Chained{0, ang_speed}
                   : chassis::diff_commands::Chained{-ang_speed, 0};

    } else {
        if (this->can_reverse ^ reverse) { // same sign
            return std::signbit(ang_speed)//3
                       ? chassis::diff_commands::Swing{-ang_speed, 0}
                       : chassis::diff_commands::Swing{0, ang_speed};
        }
        return std::signbit(ang_speed)//4
                   ? chassis::diff_commands::Swing{0, ang_speed}
                   : chassis::diff_commands::Swing{-ang_speed, 0};
    }
}

// What is calculating the required motor power for the turn
// Returns value for motor power with type double
double SwingController::angular_pid(double error) {
    total_ang_err += error;

    double speed = angular_kP * error + angular_kD * (error - prev_ang_err) +
                   angular_kI * total_ang_err;

    this->prev_ang_err = error;

    return speed;
}

// Resets all the variables used in the swing controller
void SwingController::reset() {
    this->prev_ang_err = 0;
    this->prev_ang_speed = 0;
    this->total_ang_err = 0;
    this->counter = 0;
    this->can_reverse = false;
    this->turn_overshoot = false;
}

std::shared_ptr<SwingController>
SwingController::modify_angular_constants(double kP, double kI, double kD) {
    auto pid_mod = SwingControllerBuilder::from(*this)
                       .with_angular_constants(kP, kI, kD)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<SwingController>
SwingController::modify_angular_exit_error(double exit_error) {
    auto pid_mod = SwingControllerBuilder::from(*this)
                       .with_angular_exit_error(exit_error)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<SwingController>
SwingController::modify_settle_time(double settle_time) {
    auto pid_mod = SwingControllerBuilder::from(*this)
                       .with_settle_time(settle_time)
                       .build();

    this->p = pid_mod;

    return this->p;
}

}; // namespace voss::controller
