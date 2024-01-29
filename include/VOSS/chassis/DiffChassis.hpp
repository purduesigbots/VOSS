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
    Voltages prev_voltages;

    double slew(double target, bool is_left);

  public:
    DiffChassis(std::initializer_list<int8_t> left_motors,
                std::initializer_list<int8_t> right_motors,
                controller::AbstractController& default_controller,
                double slew_step = 8);

    void tank(double left_speed, double right_speed);
    void arcade(double forward_speed, double turn_speed);

    bool execute(ChassisCommand cmd, double max);

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