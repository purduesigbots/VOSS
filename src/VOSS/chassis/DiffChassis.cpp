#include "VOSS/chassis/DiffChassis.hpp"
#include "pros/motors.h"
#include <cmath>

namespace voss::chassis {

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

DiffChassis::DiffChassis(std::initializer_list<int8_t> left_motors,
                         std::initializer_list<int8_t> right_motors,
                         controller_ptr default_controller, double slew_step)
    : AbstractChassis(default_controller) {
    this->left_motors = std::make_unique<pros::MotorGroup>(left_motors);
    this->right_motors = std::make_unique<pros::MotorGroup>(right_motors);

    this->slew_step = slew_step > 0 ? slew_step : 200;
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

bool DiffChassis::execute(DiffChassisCommand cmd, double max) {
    return std::visit(
        overload{
            [this](Stop&) -> bool {
                this->left_motors->move_voltage(0);
                this->right_motors->move_voltage(0);
                printf("Sending command(true): exit");
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
                printf("Sending command(false): %lf, %lf\n", v.left, v.right);
                return false;
            },
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