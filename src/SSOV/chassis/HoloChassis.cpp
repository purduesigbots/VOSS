#include "SSOV/chassis/HoloChassis.hpp"
#include "SSOV/routines/MoveToPoint.hpp"
#include "SSOV/routines/MoveToPose.hpp"
#include "SSOV/routines/TurnToHeading.hpp"
#include "SSOV/routines/TurnToPoint.hpp"
#include "pros/motors.h"
#include <cmath>

namespace ssov {

// Not sure what this does but was in DiffChassis

inline DriveSignal speeds_to_drive_signal(double front_left_speed, double front_right_speed, double back_left_speed, double back_right_speed) {
    // double side_speed = (front_right_speed - back_left_speed)/-2.0;
    // double turn_speed = (back_left_speed - front_right_speed)/2.0;
    // double forward_speed = (front_left_speed - side_speed - turn_speed); 
    // return {forward_speed, side_speed, turn_speed};

    double side_speed = (back_left_speed - front_left_speed)/-2.0;
    double turn_speed = (back_left_speed - front_right_speed)/2.0;
    double forward_speed = (front_left_speed - side_speed - turn_speed); 
    return {forward_speed, side_speed, turn_speed};
}

HolonomicChassis::HolonomicChassis(std::initializer_list<int8_t> front_left_mtr_pts, std::initializer_list<int8_t> front_right_mtr_pts, std::initializer_list<int8_t> back_left_mtr_pts, std::initializer_list<int8_t> back_right_mtr_pts):
front_left_motors(std::make_shared<pros::MotorGroup>(front_left_mtr_pts)),front_right_motors(std::make_shared<pros::MotorGroup>(front_right_mtr_pts)), back_left_motors(std::make_shared<pros::MotorGroup>(back_left_mtr_pts)), back_right_motors(std::make_shared<pros::MotorGroup>(back_right_mtr_pts)){
        chassis_task = pros::Task::create([this]() {
        this->task_fn();
    });
}

void HolonomicChassis::tank(double left_speed, double right_speed, double sideways_speed) {
    // double front_left = std::clamp(forward_speed + sideways_speed + turn_speed, -100.0, 100.0);
    // double front_right = std::clamp(forward_speed - sideways_speed + turn_speed, -100.0, 100.0);
    // double back_left = std::clamp(forward_speed - sideways_speed - turn_speed, -100.0, 100.0);
    // double back_right = std::clamp(forward_speed + sideways_speed - turn_speed, -100.0, 100.0);
    // current_drive_signal = speeds_to_drive_signal(front_left, front_right, back_left, back_right);
    // front_left_motors->move_voltage(front_left * 120);
    // front_right_motors->move_voltage(front_right * 120);
    // back_left_motors->move_voltage(front_left * 120);
    // back_right_motors->move_voltage(front_right * 120);
    std::cout<<"Please choose arcade"<<std::endl;
}

void HolonomicChassis::arcade(double forward_speed, double turn_speed, double sideways_speed) {
    double front_left = std::clamp(forward_speed + sideways_speed + turn_speed, -100.0, 100.0);
    double front_right = std::clamp(forward_speed - sideways_speed - turn_speed, -100.0, 100.0);
    double back_left = std::clamp(forward_speed - sideways_speed + turn_speed, -100.0, 100.0);
    double back_right = std::clamp(forward_speed + sideways_speed - turn_speed, -100.0, 100.0);
    current_drive_signal = speeds_to_drive_signal(front_left, front_right, back_left, back_right);
    this->front_left_motors->move_voltage(120.0 * front_left);
    this->front_right_motors->move_voltage(120.0 * front_right);
    this->back_left_motors->move_voltage(120.0 * back_left);
    this->back_right_motors->move_voltage(120.0 * back_right);
}

void HolonomicChassis::execute(ChassisCommand command) {
    std::visit(overloaded{
        [this](const DriveSignal &signal) {
            printf("Drive signal x: %f, y: %f, theta: %f\n", signal.x, signal.y, signal.theta);
            double front_left_speed = signal.x + signal.y + signal.theta;
            double front_right_speed = signal.x - signal.y - signal.theta;
            double back_left_speed = signal.x - signal.y + signal.theta;
            double back_right_speed =  signal.x + signal.y - signal.theta;
            double signal_max_speed = std::max(fabs(front_left_speed), std::max(fabs(front_right_speed), std::max(fabs(back_left_speed), fabs(back_right_speed))));
            double speed_scalar = max_speed / signal_max_speed;
            if (speed_scalar < 1) {
                front_left_speed *= speed_scalar;
                front_right_speed *= speed_scalar;
                back_left_speed *= speed_scalar;
                back_right_speed *= speed_scalar;
            }
            current_drive_signal = speeds_to_drive_signal(front_left_speed, front_right_speed, back_left_speed, back_right_speed);
            debug = false;
            if (debug) {
                printf("front left %f, front right %f, back left %f, back right %f\n", front_left_speed, front_right_speed, back_left_speed, back_right_speed);
            }
            front_left_motors->move_voltage(front_left_speed * 120);
            front_right_motors->move_voltage(front_right_speed * 120);
            back_left_motors->move_voltage(back_left_speed * 120);
            back_right_motors->move_voltage(back_right_speed * 120);
        }
    }, command);
}

void HolonomicChassis::task_fn() {
    while (true) {
        mtx.lock();
        if (current_routine) {
            execute(current_routine->update());
            if (current_routine->finished()) {
                current_routine = nullptr;
                max_speed = 100;
            }
        }
        mtx.unlock();
        pros::Task::current().notify_take(true, TIMEOUT_MAX);
    }
}

void HolonomicChassis::run_routine(std::shared_ptr<Routine> routine) {
    std::lock_guard<pros::Mutex> lock(mtx);
    routine->start();
    current_routine = std::move(routine);
}

void HolonomicChassis::move(Point target, PointMoveParams params) {
    MoveToPoint::Params routine_params = {
        params.controller,
        params.ec,
        localizer,
        params.slew,
        params.reverse,
        params.thru,
        params.holonomic
    };
    if (!routine_params.controller) {
        routine_params.controller = default_point_controller;
    }
    if (!routine_params.exit) {
        routine_params.exit = default_ec;
    }
    max_speed = params.max;
    routine_params.holonomic = true;
    run_routine(std::make_shared<MoveToPoint>(target, routine_params, current_drive_signal));
    //std::cout << "I am getting here" << std::endl;
    if (!params.async) {
        wait_until_done();
    }
}

void HolonomicChassis::move(UserPose target, int strafe_angle, PoseMoveParams params) {
    MoveToPose::Params routine_params = {
        params.controller,
        params.ec,
        localizer,
        params.slew,
        params.reverse,
        params.thru,
        params.holonomic
    };
    if (!routine_params.controller) {
        routine_params.controller = default_pose_controller;
    }
    if (!routine_params.ec) {
        routine_params.ec = default_ec;
    }
    max_speed = params.max;
    routine_params.holonomic = true;
    routine_params.strafe_angle = strafe_angle;
    run_routine(std::make_shared<MoveToPose>(target.to_pose(), routine_params, current_drive_signal));
    if (!params.async) {
        wait_until_done();
    }
}

void HolonomicChassis::turn(double target, TurnParams params) {
    TurnToHeading::Params routine_params = {
        params.controller,
        params.ec,
        localizer,
        params.slew,
        params.direction,
        params.thru
    };
    if (!routine_params.controller) {
        routine_params.controller = default_turn_controller;
    }
    if (!routine_params.exit) {
        routine_params.exit = default_ec;
    }
    max_speed = params.max;
    run_routine(std::make_shared<TurnToHeading>(to_radians(target), routine_params, current_drive_signal));
    if (!params.async) {
        wait_until_done();
    }
}

void HolonomicChassis::turn(Point target, TurnParams params) {
    TurnToPoint::Params routine_params = {
        params.controller,
        params.ec,
        localizer,
        params.slew,
        params.direction,
        params.reverse,
        params.thru
    };
    if (!routine_params.controller) {
        routine_params.controller = default_turn_controller;
    }
    if (!routine_params.exit) {
        routine_params.exit = default_ec;
    }
    max_speed = params.max;

    run_routine(std::make_shared<TurnToPoint>(target, routine_params, current_drive_signal));
    if (!params.async) {
        wait_until_done();
    }
}

} // namespace voss::chassis