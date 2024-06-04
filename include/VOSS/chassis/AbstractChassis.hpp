#pragma once

#include "pros/misc.hpp"
#include "pros/motors.hpp"

#include "ChassisCommand.hpp"
#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/controller/PIDController.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"

#include "VOSS/utils/flags.hpp"
#include "VOSS/utils/Point.hpp"
#include "VOSS/utils/Pose.hpp"

#include "VOSS/constants.hpp"
#include "VOSS/trajectory/SplinePath.hpp"
#include "VOSS/trajectory/Trajectory.hpp"

namespace voss::chassis {

namespace ptrs {
using controller_ptr =
    std::shared_ptr<controller::AbstractController>; // abstract controller
                                                     // pointer
using move_controller_ptr =
    std::shared_ptr<controller::IsMoveController>; // move controller pointer
using turn_controller_ptr =
    std::shared_ptr<controller::IsTurnController>; // turn controller pointer
using path_follow_controller_ptr =
    std::shared_ptr<controller::IsPathFollowController>; // path follower
                                                         // pointer
using trajectory_follow_controller_ptr = std::shared_ptr<
    controller::IsTrajectoryFollowController>; // trajectory follower pointer
using ec_ptr =
    std::shared_ptr<controller::AbstractExitCondition>; // exit condition
                                                        // pointer
} // namespace ptrs

using namespace ptrs;

class AbstractChassis {
  public:
    AbstractChassis(
        std::shared_ptr<voss::controller::PIDController> default_controller,
        std::shared_ptr<voss::localizer::AbstractLocalizer> l, ec_ptr ec);

    virtual void tank(double left_speed, double right_speed) = 0;
    virtual void arcade(double forward_speed, double turn_speed) = 0;

    virtual bool execute(DiffChassisCommand cmd, double max) = 0;
    virtual void set_brake_mode(pros::motor_brake_mode_e mode) = 0;

    void move(double distance, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE);

    void move(double distance, move_controller_ptr controller,
              double max = 100.0, voss::Flags flags = voss::Flags::NONE);

    void move(double distance, move_controller_ptr controller, ec_ptr ec,
              double max = 100.0, voss::Flags flags = voss::Flags::NONE);

    void move(Pose target, move_controller_ptr controller, ec_ptr ec,
              double max = 100.0, voss::Flags flags = voss::Flags::NONE);

    void move(Pose target, move_controller_ptr controller, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE);

    void move(Pose target, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE);

    void turn(double target, turn_controller_ptr controller, ec_ptr ec,
              double max = 100.0, voss::Flags flags = voss::Flags::NONE,
              voss::AngularDirection direction = voss::AngularDirection::AUTO);

    void turn(double target, turn_controller_ptr controller, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE,
              voss::AngularDirection direction = voss::AngularDirection::AUTO);

    void turn(double target, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE,
              voss::AngularDirection direction = voss::AngularDirection::AUTO);

    void
    turn_to(Point target, turn_controller_ptr controller, ec_ptr ec,
            double max = 100.0, voss::Flags flags = voss::Flags::NONE,
            voss::AngularDirection direction = voss::AngularDirection::AUTO);

    void
    turn_to(Point target, turn_controller_ptr controller, double max = 100.0,
            voss::Flags flags = voss::Flags::NONE,
            voss::AngularDirection direction = voss::AngularDirection::AUTO);

    void
    turn_to(Point target, double max = 100.0,
            voss::Flags flags = voss::Flags::NONE,
            voss::AngularDirection direction = voss::AngularDirection::AUTO);

    void follow_path(std::initializer_list<Pose> path,
                     path_follow_controller_ptr controller, double max = 100.0,
                     voss::FollowerFlags flags = voss::FollowerFlags::NONE);

    void follow_path(std::initializer_list<Pose> path,
                     path_follow_controller_ptr controller, ec_ptr ec,
                     double max = 100.0,
                     voss::FollowerFlags flags = voss::FollowerFlags::NONE);

    void
    follow_trajectory(const voss::trajectory::SplinePath& trajectory,
                      trajectory_follow_controller_ptr controller,
                      const voss::trajectory::TrajectoryConstraints& constraints,
                      voss::FollowerFlags flags = voss::FollowerFlags::NONE);

    void
    follow_trajectory(const voss::trajectory::SplinePath& trajectory,
                      trajectory_follow_controller_ptr controller, ec_ptr ec,
                      const voss::trajectory::TrajectoryConstraints& constraints,
                      voss::FollowerFlags flags = voss::FollowerFlags::NONE);

  protected:
    void move_task(controller_ptr controller, ec_ptr ec, double max,
                   voss::Flags flags);

    void turn_task(controller_ptr controller, ec_ptr ec, double max,
                   voss::Flags flags, voss::AngularDirection direction);

    Pose process_target_pose(Pose target, bool relative);
    double process_target_angle(double angle, bool relative);
    std::vector<Pose>
    process_target_path(const std::vector<Pose>& path);

  protected:
    std::shared_ptr<voss::controller::PIDController> default_controller;
    std::shared_ptr<voss::localizer::AbstractLocalizer> l;
    ec_ptr default_ec;
    std::unique_ptr<pros::Task> task = nullptr;
    bool task_running = false;
    pros::motor_brake_mode_e brakeMode;
};

} // namespace voss::chassis