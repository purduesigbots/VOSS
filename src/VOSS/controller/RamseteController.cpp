#include "VOSS/controller/RamseteController.hpp"
#include "VOSS/utils/angle.hpp"
#include "VOSS/utils/Math.hpp"

namespace voss::controller {

RamseteController::RamseteController(RamseteController::Params params)
    : motor_ff(params.ffwd_kS, params.ffwd_kV, params.ffwd_kA, params.ffwd_kD),
      zeta(params.zeta), b(params.b), track_width(params.track_width),
      gear_ratio(params.gear_ratio), wheel_diameter(params.wheel_diameter) {
}

std::shared_ptr<RamseteController>
RamseteController::create_controller(RamseteController::Params params) {
    return std::move(std::make_shared<RamseteController>(params));
}

std::shared_ptr<RamseteController>
controller::RamseteController::with_velocity_controller(
    voss::controller::RamseteController::PID_Params controller) {
    this->p = std::make_shared<RamseteController>(RamseteController(*this));
    this->p->left_motor_pid = {controller.left_kP, controller.left_kI,
                               controller.left_kD};
    this->p->right_motor_pid = {controller.right_kP, controller.right_kI,
                                controller.right_kD};
    return this->p;
}

chassis::DiffChassisCommand
RamseteController::get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                               std::shared_ptr<AbstractExitCondition> ec,
                               const velocity_pair& v_pair, bool reverse,
                               bool thru) {
    double current_time = (double)(pros::c::millis() - init_time) / 1000.0;
    trajectory::TrajectoryPose target_state =
        this->target_trajectory->at(current_time);
    Pose current_pose = l->get_pose();

    double x_error = target_state.pose.x - current_pose.x;
    double y_error = target_state.pose.y - current_pose.y;
    double angle_error = norm_delta(target_state.pose.theta.value() -
                                    current_pose.theta.value());
    double x_error_relative = cos(current_pose.theta.value()) * x_error +
                              sin(current_pose.theta.value()) * y_error;
    double y_error_relative = -sin(current_pose.theta.value()) * x_error +
                              cos(current_pose.theta.value()) * y_error;

    if (current_time > this->target_trajectory->get_duration()) {
        target_state.vel = 0.01 * x_error_relative;
        target_state.ang_vel = 0.01 * angle_error;
    }

    double k = 2.0 * zeta *
               sqrt(target_state.ang_vel * target_state.ang_vel +
                    b * target_state.vel * target_state.vel);

    double vel = target_state.vel * cos(angle_error) + k * x_error_relative;
    double ang_vel =
        target_state.ang_vel + k * angle_error +
        (b * target_state.vel * sin(angle_error) * y_error_relative) /
            angle_error;

    double left_vel = vel - ang_vel * track_width * 0.5;
    double right_vel = vel + ang_vel * track_width * 0.5;

    // hack: approximate left and right acceleration based on estimated left and
    // right velocity for next time step
    trajectory::TrajectoryPose lookahead =
        this->target_trajectory->at(current_time + 0.01);
    if (current_time + 0.01 > this->target_trajectory->get_duration()) {
        lookahead.vel = 0.01 * x_error_relative;
        lookahead.ang_vel = 0.01 * angle_error;
    }
    double lookahead_vel =
        lookahead.vel * cos(angle_error) + k * x_error_relative;
    double lookahead_ang_vel =
        lookahead.ang_vel + k * angle_error +
        b * lookahead.vel * sin(angle_error) * y_error_relative / angle_error;

    double left_accel =
        (lookahead_vel - lookahead_ang_vel * track_width * 0.5 - left_vel) /
        0.01;
    double right_accel =
        (lookahead_vel + lookahead_ang_vel * track_width * 0.5 - right_vel) /
        0.01;

    left_vel = motor_ff.update(left_vel, left_accel);
    right_vel = motor_ff.update(right_vel, right_accel);


//    double left_vel_ffwd = linear_to_rotational(
//        motor_ff.update(left_vel, left_accel), wheel_diameter, gear_ratio);
//    double right_vel_ffwd = linear_to_rotational(
//        motor_ff.update(right_vel, right_accel), wheel_diameter, gear_ratio);
//
//    left_vel = left_motor_pid.update(
//                   (linear_to_rotational(left_vel, wheel_diameter, gear_ratio) -
//                    v_pair.left)) +
//               left_vel_ffwd;
//    right_vel =
//        right_motor_pid.update(
//            (linear_to_rotational(right_vel, wheel_diameter, gear_ratio) -
//             v_pair.right)) +
//        right_vel_ffwd;

    if (ec->is_met(current_pose, false)) {
        return chassis::Stop{};
    }
    //    printf("%f: %lf, %lf\n", current_time, left_vel, right_vel);
    return chassis::diff_commands::Voltages{left_vel, right_vel};
}

void RamseteController::reset() {
    init_time = pros::c::millis();
}

std::shared_ptr<RamseteController>
RamseteController::modify_constants(double zeta, double b) {
    this->p = std::make_shared<RamseteController>(RamseteController(*this));
    this->p->zeta = zeta;
    this->p->b = b;

    return this->p;
}
} // namespace voss::controller