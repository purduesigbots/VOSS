#pragma once

#include "AbstractChassis.hpp"
#include "ChassisCommand.hpp"
#include "pros/motor_group.hpp"
#include <initializer_list>
#include <memory>

namespace voss::chassis {

class DiffChassis : public AbstractChassis {

  private:
    std::unique_ptr<pros::MotorGroup> left_motors;
    std::unique_ptr<pros::MotorGroup> right_motors;
    double slew_step;
    diff_commands::Voltages prev_voltages;

    double slew(double target, bool is_left);

  public:
    DiffChassis(std::initializer_list<int8_t> left_motors,
                std::initializer_list<int8_t> right_motors,
                controller_ptr default_controller, ec_ptr ec,
                double slew_step = 8,
                pros::motor_brake_mode_e brakeMode =
                    pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);

    void tank(double left_speed, double right_speed);
    void arcade(double forward_speed, double turn_speed);

    bool execute(DiffChassisCommand cmd, double max);
    void set_brake_mode(pros::motor_brake_mode_e mode) override;

    auto getMotors() const {
        struct ChassisMotorSet {
            pros::MotorGroup* left;
            pros::MotorGroup* right;
        };

        return ChassisMotorSet{this->left_motors.get(),
                               this->right_motors.get()};
    }
};

} // namespace voss::chassis