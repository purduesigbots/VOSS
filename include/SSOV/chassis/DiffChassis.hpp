#pragma once

#include <algorithm>
#include <array>
#include <initializer_list>
#include <memory>

#include "pros/motor_group.hpp"
#include "SSOV/common/Pose.hpp"

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

public:
    DiffChassis(std::initializer_list<int8_t> left_mtr_ports,
                std::initializer_list<int8_t> right_mtr_ports):
        left_mtrs(std::make_shared<pros::MotorGroup>(left_mtr_ports)),
        right_mtrs(std::make_shared<pros::MotorGroup>(right_mtr_ports)) {}
    static std::shared_ptr<DiffChassis> create(std::initializer_list<int8_t> left_mtr_ports,
                              std::initializer_list<int8_t> right_mtr_ports) {
        return std::make_shared<DiffChassis>(left_mtr_ports, right_mtr_ports);
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
};
}