#include "SSOV/chassis/DiffChassis.hpp"

#include "SSOV/routines/MoveToPoint.hpp"
#include "SSOV/routines/MoveToPose.hpp"
#include "SSOV/routines/TurnToHeading.hpp"
#include "SSOV/routines/TurnToPoint.hpp"

namespace ssov {

inline DriveSignal speeds_to_drive_signal(double left_speed, double right_speed) {
    return {(left_speed + right_speed) / 2, 0, (right_speed - left_speed) / 2};
}

DiffChassis::DiffChassis(std::initializer_list<int8_t> left_mtr_ports,
                         std::initializer_list<int8_t> right_mtr_ports,
                         std::shared_ptr<Localizer> localizer):
    left_mtrs(std::make_shared<pros::MotorGroup>(left_mtr_ports)),
    right_mtrs(std::make_shared<pros::MotorGroup>(right_mtr_ports)),
    localizer(localizer) {
    chassis_task = pros::Task::create([this]() {
        this->task_fn();
    });
    if (localizer) {
        localizer->add_listener(chassis_task);
    }
}

void DiffChassis::tank(double left_speed, double right_speed) {
    left_speed = std::clamp(left_speed, -100.0, 100.0);
    right_speed = std::clamp(right_speed, -100.0, 100.0);
    current_drive_signal = speeds_to_drive_signal(left_speed, right_speed);
    left_mtrs->move_voltage(left_speed * 120);
    right_mtrs->move_voltage(right_speed * 120);
}

void DiffChassis::arcade(double y_axis, double x_axis) {
    double left_speed = y_axis + x_axis;
    double right_speed = y_axis - x_axis;
    left_speed = std::clamp(left_speed, -100.0, 100.0);
    right_speed = std::clamp(right_speed, -100.0, 100.0);
    current_drive_signal = speeds_to_drive_signal(left_speed, right_speed);
    left_mtrs->move_voltage(left_speed * 120);
    right_mtrs->move_voltage(right_speed * 120);
}

void DiffChassis::execute(ChassisCommand command) {
    std::visit(overloaded{
        [this](const DriveSignal &signal) {
            double left_speed = signal.x - signal.theta;
            double right_speed = signal.x + signal.theta;
            double signal_max_speed = std::max(fabs(left_speed), fabs(right_speed));
            double speed_scalar = max_speed / signal_max_speed;
            if (speed_scalar < 1) {
                left_speed *= speed_scalar;
                right_speed *= speed_scalar;
            }
            current_drive_signal = speeds_to_drive_signal(left_speed, right_speed);
            left_mtrs->move_voltage(left_speed * 120);
            right_mtrs->move_voltage(right_speed * 120);
        }
    }, command);
}

void DiffChassis::task_fn() {
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

void DiffChassis::run_routine(std::shared_ptr<Routine> routine) {
    std::lock_guard<pros::Mutex> lock(mtx);
    routine->start();
    current_routine = std::move(routine);
}

void DiffChassis::move(Point target, PointMoveParams params) {
    MoveToPoint::Params routine_params = {
        params.controller,
        params.ec,
        localizer,
        params.slew,
        params.reverse,
        params.thru
    };
    if (!routine_params.controller) {
        routine_params.controller = default_point_controller;
    }
    if (!routine_params.exit) {
        routine_params.exit = default_ec;
    }
    max_speed = params.max;
    run_routine(std::make_shared<MoveToPoint>(target, routine_params, current_drive_signal));
    if (!params.async) {
        wait_until_done();
    }
}

void DiffChassis::move(Pose target, PoseMoveParams params) {
    MoveToPose::Params routine_params = {
        params.controller,
        params.ec,
        localizer,
        params.slew,
        params.reverse,
        params.thru
    };
    if (!routine_params.controller) {
        routine_params.controller = default_pose_controller;
    }
    if (!routine_params.ec) {
        routine_params.ec = default_ec;
    }
    max_speed = params.max;
    run_routine(std::make_shared<MoveToPose>(target, routine_params, current_drive_signal));
    if (!params.async) {
        wait_until_done();
    }
}

void DiffChassis::turn(double target, TurnParams params) {
    TurnToPoint::Params routine_params = {
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
    run_routine(std::make_shared<MoveToPose>(target, routine_params, current_drive_signal));
    if (!params.async) {
        wait_until_done();
    }
}

void DiffChassis::turn(Point target, TurnParams params) {
    TurnToPoint::Params routine_params = {
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
    run_routine(std::make_shared<MoveToPose>(target, routine_params, current_drive_signal));
    if (!params.async) {
        wait_until_done();
    }
}

}