#pragma once

#include "ChassisCommand.hpp"
#include "pros/motor_group.hpp"
#include <initializer_list>
#include <memory>

#include <algorithm>
#include <array>
#include "SSOV/common/Pose.hpp"
#include "SSOV/exit_condition/ExitCondition.hpp"
#include "SSOV/localizer/Localizer.hpp"
#include "SSOV/routines/Routine.hpp"
#include "SSOV/controller/PointController.hpp"
#include "SSOV/controller/PoseController.hpp"
#include "SSOV/controller/TurnController.hpp"
#include "SSOV/Defaults.hpp"

namespace ssov {

class HolonomicChassis {

  private:
    std::shared_ptr<pros::MotorGroup> front_left_motors;
    std::shared_ptr<pros::MotorGroup> front_right_motors;
    std::shared_ptr<pros::MotorGroup> back_left_motors;
    std::shared_ptr<pros::MotorGroup> back_right_motors;
    ssov::DriveSignal current_drive_signal = {};
    std::shared_ptr<Routine> current_routine = nullptr;
    double max_speed = 100;
    bool debug = false;

    void task_fn();
    pros::task_t chassis_task;
    pros::Mutex mtx;
    std::shared_ptr<Localizer> localizer;
    // to set the default controllers and exit condition last year, I just made these public,
    // but they should be set either through the constructor, or using a method.
    std::shared_ptr<PointController> default_point_controller = nullptr;
    std::shared_ptr<PoseController> default_pose_controller = nullptr;
    std::shared_ptr<TurnController> default_turn_controller = nullptr;
    std::shared_ptr<ExitCondition> default_ec = nullptr;

    double slew(double target, double direction);

  public:
    HolonomicChassis(std::initializer_list<int8_t> front_left_mtr_pts,
                std::initializer_list<int8_t> front_right_mtr_pts,
                std::initializer_list<int8_t> back_left_mtr_pts,
                std::initializer_list<int8_t> back_right_mtr_pts);
                static std::shared_ptr<HolonomicChassis> create(std::initializer_list<int8_t> front_left_mtr_pts,
                              std::initializer_list<int8_t> front_right_mtr_pts, std::initializer_list<int8_t> back_left_mtr_pts, std::initializer_list<int8_t> back_right_mtr_pts) {
                    return std::make_shared<HolonomicChassis>(front_left_mtr_pts, front_right_mtr_pts, back_left_mtr_pts, back_right_mtr_pts);
                }

    // the localizer was previously passed in the constructor, but
    // that didn't work out. Instead, the localizer should be registered
    // in initialize().
    void register_localizer(std::shared_ptr<Localizer> localizer) {
        this->localizer = localizer;
        if (localizer) {
            localizer->add_listener(chassis_task);
        }
    }

    void tank(double left_speed, double right_speed, double horizontal_speed);
    void arcade(double forward_axis, double turn_axis, double horizontal_axis);

    DriveSignal get_current_drive_signal() const {
        return current_drive_signal;
    }
    std::array<std::shared_ptr<pros::MotorGroup>, 4> get_motors() const {
        return {front_left_motors, front_right_motors, back_left_motors, back_right_motors};
    }

    void execute(ChassisCommand command);

    void run_routine(std::shared_ptr<Routine> routine);
    void wait_until_done() {
        while (current_routine) pros::delay(10);
    }
    void cancel_routine() {
        std::lock_guard<pros::Mutex> lock(mtx);
        current_routine = nullptr;
        max_speed = 100;
    }

    struct PointMoveParams {
        std::shared_ptr<PointController> controller = nullptr;
        std::shared_ptr<ExitCondition> ec = nullptr;
        double max = 100;
        double slew = defaults::slew;
        bool reverse = false;
        bool thru = false;
        bool async = false;
        bool holonomic = false;
        static PointMoveParams Default() {
            return {};
        }
    };
    void move(Point target, PointMoveParams params = PointMoveParams::Default());


    struct PoseMoveParams {
        std::shared_ptr<PoseController> controller = nullptr;
        std::shared_ptr<ExitCondition> ec = nullptr;
        double max = 100;
        double slew = defaults::slew;
        bool reverse = false;
        bool thru = false;
        bool async = false;
        bool holonomic = false;
        static PoseMoveParams Default() {
            return {};
        }
    };
    // move accepts UserPose, which has target heading in degrees,
    // and converts to radians when creating the MoveToPose routine
    void move(UserPose target, PoseMoveParams params = PoseMoveParams::Default());

    struct TurnParams {
        std::shared_ptr<TurnController> controller = nullptr;
        std::shared_ptr<ExitCondition> ec = nullptr;
        double max = 100;
        double slew = defaults::slew;
        TurnDirection direction = TurnDirection::AUTO;
        // reverse is used in turn to point, so the back of the robot
        // turns towards the point instead of the front.
        // this was used to turn the back of the robot
        // to face the wall stakes in 2025 skills
        bool reverse = false;
        bool thru = false;
        bool async = false;
        static TurnParams Default() {
            return {};
        }
    };
    // move accepts target heading in degrees and converts to radians when
    // creating the TurnToHeading routine
    void turn(double target, TurnParams params = TurnParams::Default());
    void turn(Point target, TurnParams params = TurnParams::Default());

    void set_debug(bool debug) {
        this->debug = debug;
    }
};

} // namespace voss::chassis