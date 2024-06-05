#include "VOSS/controller/RamseteController.hpp"

#include "VOSS/utils/angle.hpp"

namespace voss::controller {

RamseteController::RamseteController(RamseteController_Construct_Params params)
    : std::enable_shared_from_this<RamseteController>(),
    motor_ff(params.kS, params.kV, params.kA, params.kD), zeta(params.zeta), b(params.b), track_width(params.track_width) {
}

chassis::DiffChassisCommand
RamseteController::get_command(std::shared_ptr<localizer::AbstractLocalizer> l, bool reverse, bool thru,
                           std::shared_ptr<AbstractExitCondition> ec) {
    double current_time = (pros::c::millis() - init_time) / 1000.0;

    trajectory::TrajectoryPose target_state = this->target_trajectory->at(current_time);
    Pose current_pose = l->get_pose();

    double x_error = target_state.pose.x - current_pose.x;
    double y_error = target_state.pose.y - current_pose.y;
    double angle_error = norm_delta(target_state.pose.theta.value() - current_pose.theta.value());
    double x_error_relative = cos(current_pose.theta.value()) * x_error + sin(current_pose.theta.value()) * y_error;
    double y_error_relative = -sin(current_pose.theta.value()) * x_error + cos(current_pose.theta.value()) * y_error;

    if (target_state.vel == 0.0) {
        target_state.vel = 0.01 * x_error_relative;
        target_state.ang_vel = 0.01 * angle_error;
    }

    double k = 2 * zeta * sqrt(target_state.ang_vel * target_state.ang_vel + b * target_state.vel * target_state.vel);

    double vel = target_state.vel * cos(angle_error) + k * x_error_relative;
    double ang_vel = target_state.ang_vel + b * target_state.vel * sin(angle_error) * /*y_error*/y_error_relative / angle_error;

    double left_vel = vel - ang_vel * track_width * 0.5;
    double right_vel = vel + ang_vel * track_width * 0.5;

    left_vel = motor_ff.update(left_vel, 0.0);
    right_vel = motor_ff.update(right_vel, 0.0);

    if (ec->is_met(current_pose, thru)) {
        if (thru) {
            return chassis::diff_commands::Chained{left_vel, right_vel};
        } else {
            return chassis::Stop{};
        }
    }

    return chassis::diff_commands::Voltages{left_vel, right_vel};
}

chassis::DiffChassisCommand
RamseteController::get_angular_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                        bool reverse, bool thru,
                        voss::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) {
    return chassis::Stop{};
}

void RamseteController::reset() {
    init_time = pros::c::millis();
}

std::shared_ptr<RamseteController> RamseteController::get_ptr() {
    return this->shared_from_this();
}

std::shared_ptr<RamseteController> RamseteController::modify_constants(double zeta, double b) {
    this->p = std::make_shared<RamseteController>(RamseteController(*this));
    this->p->zeta = zeta;
    this->p->b = b;

    return this->p;
}
}