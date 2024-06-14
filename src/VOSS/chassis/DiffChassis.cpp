#include "VOSS/chassis/DiffChassis.hpp"
#include "pros/motors.h"
#include "VOSS/utils/Math.hpp"
#include <cmath>

namespace voss::chassis {

void DiffChassis::move_task(controller_ptr controller, ec_ptr ec, double max,
                            voss::Flags flags) {
    const bool reverse = flags & voss::Flags::REVERSE;
    const bool thru = flags & voss::Flags::THRU;
    auto left_v_s = this->left_motors->get_actual_velocity_all();
    auto right_v_s = this->right_motors->get_actual_velocity_all();
    ec->reset();
    controller->reset();
    printf("im here6\n");
    this->task = std::make_unique<pros::Task>([=, this]() {
        while (!this->execute(controller->get_command(
                                  this->l, ec,
                                  {average(left_v_s.begin(), left_v_s.end()),
                                   average(right_v_s.begin(), right_v_s.end())},
                                  reverse, thru),
                              max) ||
               !pros::competition::is_disabled()) {
            pros::delay(constants::MOTOR_UPDATE_DELAY);
        }
        this->task_running = false;
    });

    // Early exit for async movement
    if (flags & voss::Flags::ASYNC) {
        return;
    }

    this->task->join();
}

void DiffChassis::turn_task(controller_ptr controller, ec_ptr ec, double max,
                            voss::Flags flags,
                            voss::AngularDirection direction) {
    const bool reverse = flags & voss::Flags::REVERSE;
    const bool thru = flags & voss::Flags::THRU;
    auto left_v_s = this->left_motors->get_actual_velocity_all();
    auto right_v_s = this->right_motors->get_actual_velocity_all();
    ec->reset();
    controller->reset();
    this->task = std::make_unique<pros::Task>([=, this]() {
        while (!this->execute(controller->get_angular_command(
                                  this->l, ec,
                                  {average(left_v_s.begin(), left_v_s.end()),
                                   average(right_v_s.begin(), right_v_s.end())},
                                  reverse, thru, direction),
                              max) ||
               !pros::competition::is_disabled()) {
            pros::delay(constants::MOTOR_UPDATE_DELAY);
        }
        this->task_running = false;
    });

    // Early exit for async movement
    if (flags & voss::Flags::ASYNC) {
        return;
    }

    this->task->join();
}

// Limits acceleration by slew step
double DiffChassis::slew(double target, bool is_left) {
    double step = this->slew_step;
    double current =
        is_left ? this->prev_voltages.left : this->prev_voltages.right;

    if (fabs(current) > fabs(target))
        step = 200;

    if (target > current + step)
        current += step;
    else if (target < current - step)
        current -= step;
    else
        current = target;

    return current;
}

// Overloaded constructor for creating differential chassis with different
// controller layouts
DiffChassis::DiffChassis(
    std::initializer_list<int8_t> left_motors,
    std::initializer_list<int8_t> right_motors,
    std::shared_ptr<voss::controller::PIDController> default_controller,
    localizer_ptr localizer, ec_ptr ec,
    double slew_step, pros::motor_brake_mode_e brakeMode)
    : AbstractChassis(default_controller, localizer, ec) {
    this->left_motors = std::make_unique<pros::MotorGroup>(left_motors);
    this->right_motors = std::make_unique<pros::MotorGroup>(right_motors);

    this->slew_step = slew_step >= 0 ? slew_step : 200;
    this->brakeMode = brakeMode;
    this->left_motors->set_brake_mode(this->brakeMode);
    this->right_motors->set_brake_mode(this->brakeMode);
    this->prev_voltages = {0, 0};
}

void DiffChassis::tank(double left_speed, double right_speed) {
    this->left_motors->move_voltage(120.0 * left_speed);
    this->right_motors->move_voltage(120.0 * right_speed);
}

void DiffChassis::arcade(double forward_speed, double turn_speed) {
    double left = forward_speed + turn_speed;
    double right = forward_speed - turn_speed;

    this->left_motors->move_voltage(120.0 * left);
    this->right_motors->move_voltage(120.0 * right);
}

void DiffChassis::set_brake_mode(pros::motor_brake_mode_e mode) {
    this->brakeMode = mode;
    this->left_motors->set_brake_mode(mode);
    this->right_motors->set_brake_mode(mode);
}

// Evoke the chassis to move according to how it was set up using the
// constructor, returns true if movement is complete
bool DiffChassis::execute(DiffChassisCommand cmd, double max) {
    return std::visit(
        overload{
            [this](Stop&) -> bool {
                this->set_brake_mode(this->brakeMode);
                this->left_motors->brake();
                this->right_motors->brake();

                return true;
            },
            [this, max](diff_commands::Voltages& v) -> bool {
                double v_max = std::max(fabs(v.left), fabs(v.right));
                if (v_max > max) {
                    v.left = v.left * max / v_max;
                    v.right = v.right * max / v_max;
                }

                v.left = slew(v.left, true);
                v.right = slew(v.right, false);

                this->left_motors->move_voltage(120 * v.left);
                this->right_motors->move_voltage(120 * v.right);

                this->prev_voltages = v;

                return false;
            },
            // Logic allowing for individual movements within a chain of
            // movements to be registered at completed even though robot may
            // still be moving
            [this, max](diff_commands::Chained& v) -> bool {
                double v_max = std::max(fabs(v.left), fabs(v.right));
                if (v_max > max) {
                    v.left = v.left * max / v_max;
                    v.right = v.right * max / v_max;
                }

                v.left = slew(v.left, true);
                v.right = slew(v.right, false);

                this->left_motors->move_voltage(120 * v.left);
                this->right_motors->move_voltage(120 * v.right);

                this->prev_voltages = {v.left, v.right};

                return true;
            },
            // Logic to brake one side of the drive alloing for a turn around
            // the side of the robot and returning true when the turn is
            // finished
            [this, max](diff_commands::Swing& v) {
                double v_max = std::max(fabs(v.left), fabs(v.right));
                if (v.right == 0) {
                    this->right_motors->set_brake_mode(pros::MotorBrake::hold);
                    this->right_motors->brake();
                    if (v_max > max) {
                        v.left = v.left * max / v_max;
                    }
                    v.left = slew(v.left, true);
                    this->left_motors->move_voltage(120 * v.left);
                    this->prev_voltages = {v.left, 0.0};

                } else if (v.left == 0) {
                    this->left_motors->set_brake_mode(pros::MotorBrake::hold);
                    this->left_motors->brake();
                    if (v_max > max) {
                        v.right = v.right * max / v_max;
                    }
                    v.right = slew(v.right, false);

                    this->right_motors->move_voltage(120 * v.right);
                    this->prev_voltages = {0.0, v.right};
                }

                return false;
            }},
        cmd);
}

} // namespace voss::chassis