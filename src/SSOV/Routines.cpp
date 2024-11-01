#include "SSOV/Routines.hpp"

namespace ssov {

double slew(double target, double current, double slew) {
    if (!slew) return target;
    if (target > current + slew && current >= 0) {
        target = current + slew;
    }
    else if (target < current - slew && current <= 0) {
        target = current - slew;
    }
    return target;
}

void move(Point target, const PointMoveParams &params) {
    params.controller->reset();
    params.exit_condition->reset();
    Pose current_pose = params.localizer->get_pose();
    Pose ec_target = {target.x, target.y, current_pose.theta};
    ChassisSpeeds prev_speeds = params.chassis->get_speeds();
    pros::Task current_task = pros::Task::current();
    current_task.notify_clear();
    params.localizer->add_listener((pros::task_t)current_task);
    while (!params.exit_condition->is_met(current_pose, ec_target, params.thru)) {
        ChassisSpeeds next_speeds = params.controller->compute(current_pose, target, params.reverse, params.thru);
        ChassisSpeeds slewed_speeds = {slew(next_speeds.left_speed, prev_speeds.left_speed, params.slew), slew(next_speeds.right_speed, prev_speeds.right_speed, params.slew)};
        params.chassis->set_speeds(slewed_speeds);
        prev_speeds = slewed_speeds;
        current_task.notify_take(true, TIMEOUT_MAX);
        current_pose = params.localizer->get_pose();
        ec_target.theta = current_pose.theta;
    }
    params.chassis->tank(0, 0);
    params.localizer->remove_listener((pros::task_t)current_task);
}

}