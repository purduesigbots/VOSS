#pragma once

#include <algorithm>
#include <array>
#include <initializer_list>
#include <memory>

#include "pros/motor_group.hpp"
#include "SSOV/common/Pose.hpp"
#include "SSOV/exit_condition/ExitCondition.hpp"
#include "SSOV/localizer/Localizer.hpp"
#include "SSOV/routines/Routine.hpp"
#include "SSOV/controller/PointController.hpp"
#include "SSOV/controller/PoseController.hpp"
#include "SSOV/controller/TurnController.hpp"
#include "SSOV/Defaults.hpp"

namespace ssov {
class DiffChassis {
private:
    std::shared_ptr<pros::MotorGroup> left_mtrs;
    std::shared_ptr<pros::MotorGroup> right_mtrs;
    DriveSignal current_drive_signal = {};
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

public:
    DiffChassis(std::initializer_list<int8_t> left_mtr_ports,
                std::initializer_list<int8_t> right_mtr_ports);
    static std::shared_ptr<DiffChassis> create(std::initializer_list<int8_t> left_mtr_ports,
                              std::initializer_list<int8_t> right_mtr_ports) {
        return std::make_shared<DiffChassis>(left_mtr_ports, right_mtr_ports);
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

    void tank(double left_speed, double right_speed);
    void arcade(double y_axis, double x_axis);

    DriveSignal get_current_drive_signal() const {
        return current_drive_signal;
    }
    std::array<std::shared_ptr<pros::MotorGroup>, 2> get_motors() const {
        return {left_mtrs, right_mtrs};
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
}