#include "VOSS/controller/PIDController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>
#include <memory>
#include <utility>

namespace voss::controller {

PIDController::PIDController(PID_Construct_Params params)
    : std::enable_shared_from_this<PIDController>(),
      linear_pid(params.lin_kp, params.lin_ki, params.lin_kd),
      angular_pid(params.ang_kp, params.ang_ki, params.ang_kd),
      min_error(params.min_error), min_vel(params.min_vel) {
}

chassis::DiffChassisCommand
PIDController::get_command(Pose current_pose, bool reverse, bool thru,
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
    Point current_pos = {current_pose.x, current_pose.y};
    double current_angle = current_pose.theta.value();
    bool chainedExecutable = false;
    bool noPose = !this->target.theta.has_value();

    double dx = target.x - current_pos.x;
    double dy = target.y - current_pos.y;

    double distance_error = sqrt(dx * dx + dy * dy);

    double angle_error = atan2(dy, dx) - current_angle;

    if (reverse) {
        angle_error = atan2(-dy, -dx) - current_angle;
    }

    angle_error = voss::norm_delta(angle_error);

    double lin_speed =
        (thru ? 100.0 : (linear_pid.update(distance_error))) * dir;

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
            ang_speed = angular_pid.update(poseError);
        }

        // reduce the linear speed if the bot is tangent to the target
        lin_speed *= cos(angle_error);

    } else if (distance_error < 2 * min_error) {
        // scale angular speed down to 0 as distance_error approaches min_error
        ang_speed = angular_pid.update(angle_error);
        ang_speed *= (distance_error - min_error) / min_error;
    } else {
        if (fabs(angle_error) > M_PI_2 && this->can_reverse) {
            angle_error =
                angle_error - (angle_error / fabs(angle_error)) * M_PI;
            lin_speed = -lin_speed;
        }

        ang_speed = angular_pid.update(angle_error);
    }
    lin_speed = std::max(-100.0, std::min(100.0, lin_speed));
    // Runs at the end of a through movement
    if (ec->is_met(current_pose, thru)) {
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

chassis::DiffChassisCommand
PIDController::get_angular_command(Pose current_pose, bool reverse, bool thru,
                                   voss::AngularDirection direction,
                                   std::shared_ptr<AbstractExitCondition> ec) {
    // Runs in the background of turn commands
    // Error is the difference between the target angle and the current angle
    // ArcTan is used to find the angle between the robot and the target
    // position Output is proportional to the error and the weight of the
    // constant Controller exits when robot settled for a designated time
    // duration or accurate to a specified tolerance
    double current_angle = current_pose.theta.value();
    double target_angle = 0;
    if (!this->target.theta.has_value()) {
        Point current_pos = {current_pose.x, current_pose.y};
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
    if (ec->is_met(current_pose, thru) || chainedExecutable) {
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
}

std::shared_ptr<PIDController>
PIDController::modify_linear_constants(double kP, double kI, double kD) {
    this->p = std::make_shared<PIDController>(PIDController(*this));
    this->p->linear_pid.set_constants(kP, kI, kD);
    return this->p;
}

std::shared_ptr<PIDController>
PIDController::modify_angular_constants(double kP, double kI, double kD) {
    this->p = std::make_shared<PIDController>(PIDController(*this));
    this->p->angular_pid.set_constants(kP, kI, kD);
    return this->p;
}

std::shared_ptr<PIDController>
PIDController::modify_min_error(double min_error) {
    this->p = std::make_shared<PIDController>(PIDController(*this));
    this->p->min_error = min_error;

    return this->p;
}

std::shared_ptr<PIDController> PIDController::get_ptr() {
    return this->shared_from_this();
}

} // namespace voss::controller