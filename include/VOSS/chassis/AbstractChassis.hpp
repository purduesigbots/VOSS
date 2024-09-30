#pragma once

#include "pros/misc.hpp"
#include "pros/motors.hpp"

#include "ChassisCommand.hpp"
#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/controller/ControllerTypeTrait.hpp"
#include "VOSS/controller/PIDController.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"

#include "VOSS/utils/flags.hpp"
#include "VOSS/utils/Point.hpp"
#include "VOSS/utils/Pose.hpp"

#include "VOSS/constants.hpp"
#include "VOSS/trajectory/SplinePath.hpp"
#include "VOSS/trajectory/Trajectory.hpp"

#include "VOSS/asset/Asset.hpp"
#include "VOSS/asset/Decode.hpp"

namespace voss::ptrs {
using controller_ptr =
    std::shared_ptr<controller::AbstractController>; // abstract controller
                                                     // pointer
using ec_ptr =
    std::shared_ptr<controller::AbstractExitCondition>; // exit condition
                                                        // pointer

using localizer_ptr = std::shared_ptr<localizer::AbstractLocalizer>;
} // namespace voss::ptrs

namespace voss::chassis {
using namespace voss::ptrs;
using namespace voss::controller;

class AbstractChassis {
  public:
    AbstractChassis(
        std::shared_ptr<voss::controller::PIDController> default_controller,
        localizer_ptr l, ec_ptr ec);

    virtual bool execute(DiffChassisCommand cmd, double max) = 0;
    virtual void set_brake_mode(pros::motor_brake_mode_e mode) = 0;

    void move(double distance, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE);

    template <controller::MoveController T>
    void move(double distance, std::shared_ptr<T> controller,
              double max = 100.0, voss::Flags flags = voss::Flags::NONE);

    template <controller::MoveController T>
    void move(double distance, std::shared_ptr<T> controller, ec_ptr ec,
              double max = 100.0, voss::Flags flags = voss::Flags::NONE);

    void move(Pose target, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE);

    template <controller::MoveController T>
    void move(Pose target, std::shared_ptr<T> controller, ec_ptr ec,
              double max = 100.0, voss::Flags flags = voss::Flags::NONE);

    template <controller::MoveController T>
    void move(Pose target, std::shared_ptr<T> controller, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE);

    void turn(double target, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE,
              voss::AngularDirection direction = voss::AngularDirection::AUTO);

    template <controller::AngularController T>
    void turn(double target, std::shared_ptr<T> controller, ec_ptr ec,
              double max = 100.0, voss::Flags flags = voss::Flags::NONE,
              voss::AngularDirection direction = voss::AngularDirection::AUTO);

    template <controller::AngularController T>
    void turn(double target, std::shared_ptr<T> controller, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE,
              voss::AngularDirection direction = voss::AngularDirection::AUTO);

    void
    turn_to(Point target, double max = 100.0,
            voss::Flags flags = voss::Flags::NONE,
            voss::AngularDirection direction = voss::AngularDirection::AUTO);

    template <controller::AngularController T>
    void
    turn_to(Point target, std::shared_ptr<T> controller, ec_ptr ec,
            double max = 100.0, voss::Flags flags = voss::Flags::NONE,
            voss::AngularDirection direction = voss::AngularDirection::AUTO);

    template <controller::AngularController T>
    void
    turn_to(Point target, std::shared_ptr<T> controller, double max = 100.0,
            voss::Flags flags = voss::Flags::NONE,
            voss::AngularDirection direction = voss::AngularDirection::AUTO);

    template <controller::PathFollowController T>
    void follow_path(std::initializer_list<Pose> path,
                     std::shared_ptr<T> controller, double max = 100.0,
                     voss::FollowerFlags flags = voss::FollowerFlags::NONE);

    template <controller::PathFollowController T>
    void follow_path(std::initializer_list<Pose> path,
                     std::shared_ptr<T> controller, ec_ptr ec,
                     double max = 100.0,
                     voss::FollowerFlags flags = voss::FollowerFlags::NONE);

    template <controller::TrajectoryFollowController T>
    void follow_trajectory(
        std::initializer_list<Pose> trajectory, std::shared_ptr<T> controller,
        const voss::trajectory::TrajectoryConstraints& constraints,
        voss::FollowerFlags flags = voss::FollowerFlags::NONE);

    template <controller::TrajectoryFollowController T>
    void follow_trajectory(
        std::initializer_list<Pose> trajectory, std::shared_ptr<T> controller,
        ec_ptr ec, const voss::trajectory::TrajectoryConstraints& constraints,
        voss::FollowerFlags flags = voss::FollowerFlags::NONE);

    template <controller::TrajectoryFollowController T>
    void
    follow_trajectory(const asset& trajectory_file,
                      std::shared_ptr<T> controller,
                      voss::FollowerFlags flags = voss::FollowerFlags::NONE);

    template <controller::TrajectoryFollowController T>
    void
    follow_trajectory(const asset& trajectory_file,
                      std::shared_ptr<T> controller, ec_ptr ec,
                      voss::FollowerFlags flags = voss::FollowerFlags::NONE);

  protected:
    virtual void move_task(controller_ptr controller, ec_ptr ec, double max,
                           voss::Flags flags) = 0;

    virtual void turn_task(controller_ptr controller, ec_ptr ec, double max,
                           voss::Flags flags,
                           voss::AngularDirection direction) = 0;

    Pose process_target_pose(Pose target, bool relative);
    double process_target_angle(double angle, bool relative);
    std::vector<Pose> process_target_path(const std::vector<Pose>& path);

  protected:
    std::shared_ptr<voss::controller::PIDController> default_controller;
    localizer_ptr l;
    ec_ptr default_ec;
    std::unique_ptr<pros::Task> task = nullptr;
    bool task_running = false;
    pros::motor_brake_mode_e brakeMode;
};

// template impl

template <MoveController T>
void AbstractChassis::move(double distance, std::shared_ptr<T> controller,
                           double max, voss::Flags flags) {
    printf("im here1\n");
    this->move({distance, 0}, std::move(controller), this->default_ec, max,
               flags | voss::Flags::RELATIVE);
}

template <MoveController T>
void AbstractChassis::move(double distance, std::shared_ptr<T> controller,
                           ec_ptr ec, double max, voss::Flags flags) {
    printf("im here2\n");
    this->move({distance, 0}, std::move(controller), std::move(ec), max,
               flags | Flags::RELATIVE);
}

template <MoveController T>
void AbstractChassis::move(Pose target, std::shared_ptr<T> controller,
                           double max, voss::Flags flags) {
    printf("im here4\n");
    this->move(target, std::move(controller), this->default_ec, max, flags);
}

template <MoveController T>
void AbstractChassis::move(Pose target, std::shared_ptr<T> controller,
                           ec_ptr ec, double max, voss::Flags flags) {
    printf("im here5\n");
    while (this->task_running) {
        pros::delay(constants::MOTOR_UPDATE_DELAY);
    }
    this->task_running = true;

    Pose processed_target =
        this->process_target_pose(target, flags & voss::Flags::RELATIVE);
    printf("im here7\n");
    printf("%lf, %lf, %lf", processed_target.x, processed_target.y,
           processed_target.theta.value_or(0.0));
    controller->set_target_pose(processed_target);
    ec->set_target(processed_target);
    printf("im here8\n");
    this->move_task(std::move(controller), std::move(ec), max, flags);
}

template <AngularController T>
void AbstractChassis::turn(double target, std::shared_ptr<T> controller,
                           double max, voss::Flags flags,
                           voss::AngularDirection direction) {
    this->turn(target, std::move(controller), this->default_ec, max, flags,
               direction);
}

template <AngularController T>
void AbstractChassis::turn(double target, std::shared_ptr<T> controller,
                           ec_ptr ec, double max, voss::Flags flags,
                           voss::AngularDirection direction) {
    while (this->task_running) {
        pros::delay(constants::MOTOR_UPDATE_DELAY);
    }
    this->task_running = true;

    double processed_target =
        this->process_target_angle(target, flags & voss::Flags::RELATIVE);

    controller->set_target_pose({NAN, NAN, processed_target});
    controller->set_target_angle(processed_target);
    ec->set_target({NAN, NAN, processed_target});

    this->turn_task(std::move(controller), std::move(ec), max, flags,
                    direction);
}

template <AngularController T>
void AbstractChassis::turn_to(Point target, std::shared_ptr<T> controller,
                              double max, voss::Flags flags,
                              voss::AngularDirection direction) {
    this->turn_to(target, std::move(controller), this->default_ec, max, flags,
                  direction);
}

template <AngularController T>
void AbstractChassis::turn_to(Point target, std::shared_ptr<T> controller,
                              ec_ptr ec, double max, voss::Flags flags,
                              voss::AngularDirection direction) {
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;

    Pose processed_target = process_target_pose(
        {target.x, target.y, std::nullopt}, flags & voss::Flags::RELATIVE);

    controller->set_target_pose(processed_target);
    ec->set_target(processed_target);

    this->turn_task(std::move(controller), std::move(ec), max, flags,
                    direction);
}

template <PathFollowController T>
void AbstractChassis::follow_path(std::initializer_list<Pose> path,
                                  std::shared_ptr<T> controller, double max,
                                  voss::FollowerFlags flags) {
    this->follow_path(path, std::move(controller), this->default_ec, max,
                      flags);
}

template <PathFollowController T>
void AbstractChassis::follow_path(std::initializer_list<Pose> path,
                                  std::shared_ptr<T> controller, ec_ptr ec,
                                  double max, voss::FollowerFlags flags) {
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;
    auto processed_path = this->process_target_path(path);

    controller->set_target_path(processed_path);
    ec->set_target(*(processed_path.end() - 1));

    this->move_task(std::move(controller), std::move(ec), max,
                    static_cast<Flags>(flags));
}

template <TrajectoryFollowController T>
void AbstractChassis::follow_trajectory(
    std::initializer_list<Pose> trajectory, std::shared_ptr<T> controller,
    const voss::trajectory::TrajectoryConstraints& constraints,
    voss::FollowerFlags flags) {
    this->follow_trajectory(trajectory, std::move(controller), this->default_ec,
                            constraints, flags);
}

template <TrajectoryFollowController T>
void AbstractChassis::follow_trajectory(
    std::initializer_list<Pose> trajectory, std::shared_ptr<T> controller,
    ec_ptr ec, const voss::trajectory::TrajectoryConstraints& constraints,
    voss::FollowerFlags flags) {
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;

    auto processed_traj = this->process_target_path(trajectory);
    auto target_traj = trajectory::Trajectory(
        {processed_traj, flags & voss::FollowerFlags::REVERSE}, constraints);

    controller->set_target_trajectory(target_traj);
    ec->set_target(*(trajectory.end() - 1));

    this->move_task(std::move(controller), std::move(ec), 100,
                    static_cast<Flags>(flags));
}

template <TrajectoryFollowController T>
void AbstractChassis::follow_trajectory(const asset& trajectory_file,
                                        std::shared_ptr<T> controller,
                                        voss::FollowerFlags flags) {
    this->follow_trajectory(trajectory_file, std::move(controller),
                            this->default_ec, flags);
}

template <TrajectoryFollowController T>
void AbstractChassis::follow_trajectory(const asset& trajectory_file,
                                        std::shared_ptr<T> controller,
                                        ec_ptr ec, voss::FollowerFlags flags) {
    while (this->task_running) {
        pros::delay(10);
    }
    this->task_running = true;

    auto target_traj = decode_csv(trajectory_file);

    controller->set_target_trajectory(target_traj);
    ec->set_target(target_traj.at(target_traj.get_duration()).pose);

    this->move_task(std::move(controller), std::move(ec), 100,
                    static_cast<Flags>(flags));
}

} // namespace voss::chassis