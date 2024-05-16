#include "VOSS/controller/SwingController.hpp"

#include "VOSS/utils/angle.hpp"
#include <utility>

namespace voss::controller {
SwingController::SwingController(SwingController_Construct_Params params)
    : std::enable_shared_from_this<SwingController>(),
      angular_pid(params.ang_kp, params.ang_ki, params.ang_kd){};

chassis::DiffChassisCommand
SwingController::get_command(std::shared_ptr<localizer::AbstractLocalizer> l, bool reverse, bool thru,
                             std::shared_ptr<AbstractExitCondition> ec) {
    return chassis::DiffChassisCommand{chassis::Stop{}};
}

chassis::DiffChassisCommand SwingController::get_angular_command(
    std::shared_ptr<localizer::AbstractLocalizer> l, bool reverse, bool thru,
    voss::AngularDirection direction,
    std::shared_ptr<AbstractExitCondition> ec) {
    // Runs in background of Swing Turn commands
    // ArcTan is used to find the angle between the robot and the target
    // position One size of the drive is locked in place while the other side
    // moves. This is determined by which side has less required movement
    // Reverse flag flips the direction the moving side
    // Power appled to motors based on proportion of the error and the weight of
    // the constant Exit conditions are when the robot has settled for a
    // designated time duration or accurate to a specified tolerance
    double current_angle = l->get_orientation_rad();
    double target_angle = 0;
    if (!this->target.theta.has_value()) {
        Point current_pos = l->get_position();
        double dx = this->target.x - current_pos.x;
        double dy = this->target.y - current_pos.y;
        target_angle = atan2(dy, dx);
    } else {
        target_angle = this->angular_target;
    }
    double angular_error = target_angle - current_angle;

    angular_error = voss::norm_delta(angular_error);
    bool chainedExecutable = false;
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
    chassis::DiffChassisCommand command;
    if (!((ang_speed >= 0.0) ^ (this->prev_ang_speed < 0.0)) &&
        this->prev_ang_speed != 0) {
        can_reverse = !can_reverse;
    }

    prev_ang_speed = ang_speed;

    if (ec->is_met(l->get_pose(), thru) || chainedExecutable) { // exit or thru
        if (thru) {
            if (this->can_reverse ^ !reverse) { // same sign
                return std::signbit(ang_speed)  // 1
                           ? chassis::diff_commands::Chained{-ang_speed, 0}
                           : chassis::diff_commands::Chained{0, ang_speed};
            }
            return std::signbit(ang_speed) // 2
                       ? chassis::diff_commands::Chained{0, ang_speed}
                       : chassis::diff_commands::Chained{-ang_speed, 0};
        }
        return chassis::Stop{};
    }

    if (this->can_reverse ^ !reverse) { // same sign
        return std::signbit(ang_speed)  // 3
                   ? chassis::diff_commands::Swing{-ang_speed, 0}
                   : chassis::diff_commands::Swing{0, ang_speed};
    }
    return std::signbit(ang_speed) // 4
               ? chassis::diff_commands::Swing{0, ang_speed}
               : chassis::diff_commands::Swing{-ang_speed, 0};
}

// Resets all the variables used in the swing controller
void SwingController::reset() {
    this->angular_pid.reset();
    this->prev_ang_speed = 0;
    this->can_reverse = false;
    this->turn_overshoot = false;
}

std::shared_ptr<SwingController>
SwingController::modify_angular_constants(double kP, double kI, double kD) {
    this->p = std::make_shared<SwingController>(SwingController(*this));

    this->p->angular_pid.set_constants(kP, kI, kD);

    return this->p;
}

std::shared_ptr<SwingController> SwingController::get_ptr() {
    return this->shared_from_this();
}

}; // namespace voss::controller
