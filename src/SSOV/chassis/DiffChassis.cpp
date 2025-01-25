#include "SSOV/chassis/DiffChassis.hpp"

#include "SSOV/routines/MoveToPoint.hpp"

namespace ssov {

DiffChassis::DiffChassis(std::initializer_list<int8_t> left_mtr_ports,
                         std::initializer_list<int8_t> right_mtr_ports,
                         std::shared_ptr<Localizer> localizer):
    left_mtrs(std::make_shared<pros::MotorGroup>(left_mtr_ports)),
    right_mtrs(std::make_shared<pros::MotorGroup>(right_mtr_ports)),
    localizer(localizer) {
    chassis_task = pros::Task::create([this]() {
        this->task_fn();
    });
    localizer->add_listener(chassis_task);
}

void DiffChassis::execute(ChassisCommand command) {
    std::visit(overloaded{
        [this](const DriveSignal &signal) {
            printf("drive signal %f %f %f\n", signal.x, signal.y, signal.theta);
            tank(signal.x - signal.theta, signal.x + signal.theta);
        }
    }, command);
}

void DiffChassis::task_fn() {
    while (true) {
        mtx.lock();
        Pose current_pose = localizer->get_pose();
        if (current_routine) {
            printf("%f %f %f\n", current_pose.x, current_pose.y, current_pose.theta);
            execute(current_routine->update());
            if (current_routine->finished()) {
                std::cout << "finished\n";
                current_routine = nullptr;
            }
        }
        mtx.unlock();
        pros::Task::current().notify_take(true, TIMEOUT_MAX);
    }
}

void DiffChassis::run_routine(std::shared_ptr<Routine> routine) {
    std::lock_guard<pros::Mutex> lock(mtx);
    current_routine = std::move(routine);
}

void DiffChassis::move(Point target) {
    DriveSignal current_signal = {
        (left_speed + right_speed) / 2,
        0,
        (right_speed - left_speed) / 2
    };
    run_routine(std::make_shared<MoveToPoint>(target, MoveToPoint::Params{default_point_controller, default_ec, localizer, 8, false, false}, current_signal));
}

}