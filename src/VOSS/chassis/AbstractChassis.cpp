#include "VOSS/chassis/AbstractChassis.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "VOSS/constants.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/utils/angle.hpp"

#include <cmath>

namespace voss::chassis {

AbstractChassis::AbstractChassis(
    std::shared_ptr<voss::controller::PIDController> default_controller,
    std::shared_ptr<voss::localizer::AbstractLocalizer> l, ec_ptr ec) {
    this->l = std::move(l);
    this->default_controller = std::move(default_controller);
    this->default_ec = std::move(ec);
}

void AbstractChassis::move_task(controller_ptr controller, ec_ptr ec,
                                double max, voss::Flags flags) {
    const bool reverse = flags & voss::Flags::REVERSE;
    const bool thru = flags & voss::Flags::THRU;

    ec->reset();
    controller->reset();

    this->task = std::make_unique<pros::Task>([=, this]() {
        while (!this->execute(
                   controller->get_command(this->l, reverse, thru, ec), max) ||
               !pros::competition::is_disabled()) {
            pros::delay(constants::MOTOR_UPDATE_DELAY);
        }
        this->task_running = false;
    });

    // Early exit for async movement
    if (flags & voss::Flags::ASYNC) {
        return;
    }

    this->task->join();
}

void AbstractChassis::turn_task(controller_ptr controller, ec_ptr ec,
                                double max, voss::Flags flags,
                                voss::AngularDirection direction) {
    const bool reverse = flags & voss::Flags::REVERSE;
    const bool thru = flags & voss::Flags::THRU;
    ec->reset();
    controller->reset();
    this->task = std::make_unique<pros::Task>([=, this]() {
        while (!this->execute(controller->get_angular_command(
                                  this->l, reverse, thru, direction, ec),
                              max) ||
               !pros::competition::is_disabled()) {
            pros::delay(constants::MOTOR_UPDATE_DELAY);
        }
        this->task_running = false;
    });

    // Early exit for async movement
    if (flags & voss::Flags::ASYNC) {
        return;
    }

    this->task->join();
}

void AbstractChassis::move(double distance, double max, voss::Flags flags) {
    this->move({distance, 0}, this->default_controller, this->default_ec, max,
               flags | voss::Flags::RELATIVE);
}

void AbstractChassis::move(double distance, move_controller_ptr controller,
                           double max, voss::Flags flags) {

    this->move({distance, 0}, std::move(controller), this->default_ec, max,
               flags | voss::Flags::RELATIVE);
}

void AbstractChassis::move(double distance, move_controller_ptr controller,
                           ec_ptr ec, double max, voss::Flags flags) {

    this->move({distance, 0}, std::move(controller), std::move(ec), max,
               flags | Flags::RELATIVE);
}

void AbstractChassis::move(Pose target, double max, voss::Flags flags) {
    this->move(target, this->default_controller, this->default_ec, max, flags);
}

void AbstractChassis::move(Pose target, move_controller_ptr controller,
                           double max, voss::Flags flags) {

    this->move(target, std::move(controller), this->default_ec, max, flags);
}

void AbstractChassis::move(Pose target, move_controller_ptr controller,
                           ec_ptr ec, double max, voss::Flags flags) {

    while (this->task_running) {
        pros::delay(constants::MOTOR_UPDATE_DELAY);
    }
    this->task_running = true;

    Pose processed_target =
        this->process_target_pose(target, flags & voss::Flags::RELATIVE);

    controller->set_target(processed_target);
    ec->set_target(processed_target);

    this->move_task(std::move(controller), std::move(ec), max, flags);
}

void AbstractChassis::turn(double target, double max, voss::Flags flags,
                           voss::AngularDirection direction) {
    this->turn(target, this->default_controller, this->default_ec, max, flags,
               direction);
}

void AbstractChassis::turn(double target, turn_controller_ptr controller,
                           double max, voss::Flags flags,
                           voss::AngularDirection direction) {
    this->turn(target, std::move(controller), this->default_ec, max, flags,
               direction);
}

void AbstractChassis::turn(double target, turn_controller_ptr controller,
                           ec_ptr ec, double max, voss::Flags flags,
                           voss::AngularDirection direction) {
    while (this->task_running) {
        pros::delay(constants::MOTOR_UPDATE_DELAY);
    }
    this->task_running = true;

    double processed_target =
        this->process_target_angle(target, flags & voss::Flags::RELATIVE);

    controller->set_target({NAN, NAN, processed_target});
    controller->set_angular_target(processed_target);
    ec->set_target({NAN, NAN, processed_target});

    this->turn_task(std::move(controller), std::move(ec), max, flags,
                    direction);
}

void AbstractChassis::turn_to(Point target, double max, voss::Flags flags,
                              voss::AngularDirection direction) {
    this->turn_to(target, this->default_controller, this->default_ec, max,
                  flags, direction);
}

void AbstractChassis::turn_to(Point target, turn_controller_ptr controller,
                              double max, voss::Flags flags,
                              voss::AngularDirection direction) {
    this->turn_to(target, std::move(controller), this->default_ec, max, flags,
                  direction);
}

void AbstractChassis::turn_to(Point target, turn_controller_ptr controller,
                              ec_ptr ec, double max, voss::Flags flags,
                              voss::AngularDirection direction) {
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;

    Pose processed_target = process_target_pose(
        {target.x, target.y, std::nullopt}, flags & voss::Flags::RELATIVE);

    controller->set_target(processed_target);
    ec->set_target(processed_target);

    this->turn_task(std::move(controller), std::move(ec), max, flags,
                    direction);
}
void AbstractChassis::follow_path(std::initializer_list<Pose> path,
                                  path_follow_controller_ptr controller,
                                  double max, voss::FollowerFlags flags) {
    this->follow_path(path, std::move(controller), this->default_ec, max,
                      flags);
}

void AbstractChassis::follow_path(std::initializer_list<Pose> path,
                                  path_follow_controller_ptr controller,
                                  ec_ptr ec, double max,
                                  voss::FollowerFlags flags) {
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;
    auto processed_path = this->process_target_path(path);

    controller->set_target_path(processed_path);
    ec->set_target(*processed_path.end());

    this->move_task(std::move(controller), std::move(ec), max,
                    static_cast<Flags>(flags));
}

void AbstractChassis::follow_trajectory(
    const trajectory::SplinePath& trajectory,
    trajectory_follow_controller_ptr controller,
    const voss::trajectory::TrajectoryConstraints& constraints,
    voss::FollowerFlags flags) {
    this->follow_trajectory(trajectory, std::move(controller), this->default_ec,
                            constraints, flags);
}

void AbstractChassis::follow_trajectory(
    const trajectory::SplinePath& trajectory,
    trajectory_follow_controller_ptr controller, ec_ptr ec,
    const voss::trajectory::TrajectoryConstraints& constraints,
    voss::FollowerFlags flags) {
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;

    auto target_traj = trajectory::Trajectory(trajectory, constraints);

    controller->set_target_trajectory(target_traj);
    ec->set_target(target_traj.at(1).pose.pose);

    this->move_task(std::move(controller), std::move(ec), 100,
                    static_cast<Flags>(flags));
}

Pose AbstractChassis::process_target_pose(Pose target, bool relative) {
    if (target.theta.has_value()) {
        target.theta = voss::to_radians(target.theta.value());
    }
    if (relative) {
        return Pose::get_relative(target, this->l->get_pose());
    }
    return target;
}

double AbstractChassis::process_target_angle(double angle, bool relative) {
    angle = voss::to_radians(angle);
    if (relative) {
        return voss::norm(angle + this->l->get_orientation_rad());
    }
    return voss::norm(angle);
}

std::vector<Pose>
AbstractChassis::process_target_path(const std::vector<Pose>& path) {
    for (auto it : path) {
        if (it.theta.has_value()) {
            it.theta = voss::to_radians(it.theta.value());
        }
    }
    return path;
}

} // namespace voss::chassis