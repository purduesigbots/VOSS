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

namespace ssov {
struct ChassisSpeeds {
    double left_speed;
    double right_speed;
};
class DiffChassis {
private:
    std::shared_ptr<pros::MotorGroup> left_mtrs;
    std::shared_ptr<pros::MotorGroup> right_mtrs;
    double left_speed = 0.0;
    double right_speed = 0.0;
    std::shared_ptr<Routine> current_routine = nullptr;
    void task_fn();
    pros::task_t chassis_task;
    pros::Mutex mtx;
    std::shared_ptr<Localizer> localizer;

public:
    DiffChassis(std::initializer_list<int8_t> left_mtr_ports,
                std::initializer_list<int8_t> right_mtr_ports,
                std::shared_ptr<Localizer> localizer);
    static std::shared_ptr<DiffChassis> create(std::initializer_list<int8_t> left_mtr_ports,
                              std::initializer_list<int8_t> right_mtr_ports,
                              std::shared_ptr<Localizer> localizer) {
        return std::make_shared<DiffChassis>(left_mtr_ports, right_mtr_ports, localizer);
    }
    void tank(double left_speed, double right_speed) {
        left_speed = std::clamp(left_speed, -100.0, 100.0);
        right_speed = std::clamp(right_speed, -100.0, 100.0);
        this->left_speed = left_speed;
        this->right_speed = right_speed;
        left_mtrs->move_voltage(left_speed * 120);
        right_mtrs->move_voltage(right_speed * 120);
    }
    void arcade(double forward_speed, double turn_speed) {
        left_speed = forward_speed + turn_speed;
        right_speed = forward_speed - turn_speed;
        left_speed = std::clamp(left_speed, -100.0, 100.0);
        right_speed = std::clamp(right_speed, -100.0, 100.0);
        left_mtrs->move_voltage(left_speed * 120);
        right_mtrs->move_voltage(right_speed * 120);
    }
    void set_speeds(const ChassisSpeeds& speeds) {
        tank(speeds.left_speed, speeds.right_speed);
    }
    ChassisSpeeds get_speeds() const {
        return {left_speed, right_speed};
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
    }

    void move(Point target);
    std::shared_ptr<PointController> default_point_controller = nullptr;
    std::shared_ptr<ExitCondition> default_ec = nullptr;
};
}