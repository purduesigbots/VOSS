#pragma once

#include "pros/misc.hpp"
#include "pros/motors.hpp"

#include "ChassisCommand.hpp"
#include "VOSS/controller/AbstractController.hpp"

#include "VOSS/utils/flags.hpp"
#include "VOSS/utils/Point.hpp"
#include "VOSS/utils/Pose.hpp"

namespace voss::chassis {

using controller_ptr = std::shared_ptr<controller::AbstractController>;

class AbstractChassis {
  protected:
    controller_ptr default_controller;
    std::unique_ptr<pros::Task> task = nullptr;
    bool task_running = false;
    pros::motor_brake_mode_e brakeMode;


    void move_task(controller_ptr controller, double max, voss::Flags flags,
                   double exitTime);

    void turn_task(controller_ptr controller, double max, voss::Flags flags,
                   voss::AngularDirection direction, double exitTime);

  public:
    AbstractChassis(controller_ptr default_controller);

    virtual void tank(double left_speed, double right_speed) = 0;
    virtual void arcade(double forward_speed, double turn_speed) = 0;

    virtual bool execute(DiffChassisCommand cmd, double max) = 0;
    virtual void set_brake_mode(pros::motor_brake_mode_e mode) = 0;

    void move(Point target, controller_ptr controller, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE, double exitTime = 22500);
    void move(Pose target, controller_ptr controller, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE, double exitTime = 22500);
    void move(Point target, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE, double exitTime = 22500);
    void move(Pose target, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE, double exitTime = 22500);

    void turn(double target, controller_ptr controller, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE,
              voss::AngularDirection direction = voss::AngularDirection::AUTO,
              double exitTime = 22500);
    void turn(double target, double max = 100.0,
              voss::Flags flags = voss::Flags::NONE,
              voss::AngularDirection direction = voss::AngularDirection::AUTO,
              double exitTime = 22500);
    void
    turn_to(Point target, controller_ptr controller, double max = 100.0,
            voss::Flags flags = voss::Flags::NONE,
            voss::AngularDirection direction = voss::AngularDirection::AUTO,
            double exitTime = 22500);
    void
    turn_to(Point target, double max = 100.0,
            voss::Flags flags = voss::Flags::NONE,
            voss::AngularDirection direction = voss::AngularDirection::AUTO,
            double exitTime = 22500);
};

} // namespace voss::chassis