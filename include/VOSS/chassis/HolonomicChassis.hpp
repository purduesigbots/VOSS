#pragma once

#include "AbstractChassis.hpp"
#include "ChassisCommand.hpp"
#include "pros/motor_group.hpp"
#include <initializer_list>
#include <memory>

namespace voss::chassis {

class HolonomicChassis : public AbstractChassis {

  private:
    std::unique_ptr<pros::MotorGroup> front_left_motors;
    std::unique_ptr<pros::MotorGroup> front_right_motors;
    std::unique_ptr<pros::MotorGroup> back_left_motors;
    std::unique_ptr<pros::MotorGroup> back_right_motors;
    double slew_step;
    holo_commands::Voltages prev_voltages;

    double slew(double target, double direction);

  public:
    HolonomicChassis(std::initializer_list<int8_t> front_left_motors,
                std::initializer_list<int8_t> front_right_motors,
                std::initializer_list<int8_t> back_left_motors,
                std::initializer_list<int8_t> back_right_motors,
                controller_ptr default_controller, ec_ptr ec,
                double slew_step = 8,
                pros::motor_brake_mode_e brakeMode =
                    pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);

    void holonomic(double forward_speed, double sideways_speed, double turn_speed);

    bool execute(HoloChassisCommand cmd, double max);
    void set_brake_mode(pros::motor_brake_mode_e mode) override;

    auto getMotors() const {
        struct ChassisMotorSet {
            pros::MotorGroup* front_left;
            pros::MotorGroup* front_right;
            pros::MotorGroup* back_left;
            pros::MotorGroup* back_right;
        };

        return ChassisMotorSet{this->front_left_motors.get(),
                               this->front_right_motors.get(),
                               this->back_left_motors.get(),
                               this->back_right_motors.get()};
    }
};

} // namespace voss::chassis