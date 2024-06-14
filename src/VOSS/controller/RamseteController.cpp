#include "VOSS/controller/RamseteController.hpp"

#include "VOSS/utils/angle.hpp"

namespace voss::controller {

RamseteController::RamseteController(RamseteController::Params params)
    : motor_ff(params.kS, params.kV, params.kA, params.kD), zeta(params.zeta), b(params.b), track_width(params.track_width) {
}

chassis::DiffChassisCommand
RamseteController::get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                               std::shared_ptr<AbstractExitCondition> ec,
                               const velocity_pair& v_pair, bool reverse, bool thru) {
    double current_time = (pros::c::millis() - init_time) / 1000.0;

    trajectory::TrajectoryPose target_state = this->target_trajectory->at(current_time);
    Pose current_pose = l->get_pose();

    double x_error = target_state.pose.x - current_pose.x;
    double y_error = target_state.pose.y - current_pose.y;
    double angle_error = norm_delta(target_state.pose.theta.value() - current_pose.theta.value());
    double x_error_relative = cos(current_pose.theta.value()) * x_error + sin(current_pose.theta.value()) * y_error;
    double y_error_relative = -sin(current_pose.theta.value()) * x_error + cos(current_pose.theta.value()) * y_error;

    if (current_time > this->target_trajectory->get_duration()) {
        target_state.vel = 0.01 * x_error_relative;
        target_state.ang_vel = 0.01 * angle_error;
    }

    double k = 2 * zeta * sqrt(target_state.ang_vel * target_state.ang_vel + b * target_state.vel * target_state.vel);

    double vel = target_state.vel * cos(angle_error) + k * x_error_relative;
    double ang_vel = target_state.ang_vel + k * angle_error + b * target_state.vel * sin(angle_error) * y_error_relative / angle_error;

    double left_vel = vel - ang_vel * track_width * 0.5;
    double right_vel = vel + ang_vel * track_width * 0.5;

    // hack: approximate left and right acceleration based on estimated left and right velocity for next time step
    trajectory::TrajectoryPose lookahead = this->target_trajectory->at(current_time + 0.01);
    if (current_time + 0.01 > this->target_trajectory->get_duration()) {
        lookahead.vel = 0.01 * x_error_relative;
        lookahead.ang_vel = 0.01 * angle_error;
    }
    double lookahead_vel = lookahead.vel * cos(angle_error) + k * x_error_relative;
    double lookahead_ang_vel = lookahead.ang_vel + k * angle_error + b * lookahead.vel * sin(angle_error) * y_error_relative / angle_error;

    double left_accel = (lookahead_vel - lookahead_ang_vel * track_width * 0.5 - left_vel) / 0.01;
    double right_accel = (lookahead_vel + lookahead_ang_vel * track_width * 0.5 - right_vel) / 0.01;

    left_vel = motor_ff.update(left_vel, left_accel);
    right_vel = motor_ff.update(right_vel, right_accel);

    if (ec->is_met(current_pose, thru)) {
        if (thru) {
            return chassis::diff_commands::Chained{left_vel, right_vel};
        } else {
            return chassis::Stop{};
        }
    }

    return chassis::diff_commands::Voltages{left_vel, right_vel};
}

void RamseteController::reset() {
    init_time = pros::c::millis();
}

std::shared_ptr<RamseteController> RamseteController::modify_constants(double zeta, double b) {
    this->p = std::make_shared<RamseteController>(RamseteController(*this));
    this->p->zeta = zeta;
    this->p->b = b;

    return this->p;
}
}