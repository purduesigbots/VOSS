#include "VOSS/controller/SwingController.hpp"
#include "VOSS/utils/angle.hpp"

namespace voss::controller {
SwingController::SwingController(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : AbstractController(l){};

chassis::ChassisCommand SwingController::get_command(bool reverse, bool thru) {
    return chassis::ChassisCommand{chassis::Stop{}};
}

chassis::ChassisCommand
SwingController::get_angular_command(bool reverse, bool thru,
                                     voss::AngularDirection direction) {
    //Runs in background of Swing Turn commands
    //ArcTan is used to find the angle between the robot and the target position
    //One size of the drive is locked in place while the other side moves. This is determined by which side has less required movement
    //Reverse flag flips the direction the moving side
    //Power appled to motors based on proportion of the error and the weight of the constant
    //Exit conditions are when the robot has settled for a designated time duration or accurate to a specified tolerance
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
        return chassis::ChassisCommand{chassis::Stop{}};
    }
    if (fabs(angular_error - prev_ang_err) < voss::to_radians(0.1) &&
        counter > 400) {
        close_2 += 10;
    } else {
        close_2 = 0;
    }

    if (close_2 > settle_time * 2) {
        return chassis::ChassisCommand{chassis::Stop{}};
    }

    double ang_speed = angular_pid(angular_error);
    chassis::ChassisCommand command;
    if (!((ang_speed >= 0.0) ^ (this->prev_ang_speed < 0.0)) &&
        this->prev_ang_speed != 0) {
        can_reverse = !can_reverse;
    }

    if (!this->can_reverse) {
        if (reverse) {
            command = std::signbit(ang_speed) ? chassis::Swing{-ang_speed, 0}
                                              : chassis::Swing{0, ang_speed};
        } else {
            command = std::signbit(ang_speed) ? chassis::Swing{0, ang_speed}
                                              : chassis::Swing{-ang_speed, 0};
        }
    } else {
        if (!reverse) {
            command = std::signbit(ang_speed) ? chassis::Swing{-ang_speed, 0}
                                              : chassis::Swing{0, ang_speed};
        } else {
            command = std::signbit(ang_speed) ? chassis::Swing{0, ang_speed}
                                              : chassis::Swing{-ang_speed, 0};
        }
    }

    prev_ang_speed = ang_speed;

    return command;
}


//What is calculating the required motor power for the turn
//Returns value for motor power with type double
double SwingController::angular_pid(double error) {
    total_ang_err += error;

    double speed = angular_kP * error + angular_kD * (error - prev_ang_err) +
                   angular_kI * total_ang_err;

    this->prev_ang_err = error;

    return speed;
}

//Resets all the variables used in the swing controller
void SwingController::reset() {
    this->prev_ang_err = 0;
    this->prev_ang_speed = 0;
    this->total_ang_err = 0;
    this->counter = 0;
    this->can_reverse = false;
    this->turn_overshoot = false;
}
}; // namespace voss::controller
