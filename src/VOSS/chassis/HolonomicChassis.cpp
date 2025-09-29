#include "VOSS/chassis/HolonomicChassis.hpp"
#include "pros/motors.h"
#include <cmath>

namespace voss::chassis {

// Limits acceleration by slew step
double HolonomicChassis::slew(double target, double direction) {
    double step = this->slew_step;
    double current = 0;

    if(direction == 0){
        current = prev_voltages.v_x;
    }else if(direction == 1){
        current = prev_voltages.v_y;
    }else if(direction == 2){
        current = prev_voltages.v_theta;
    }else{
        return 0;
    }

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
HolonomicChassis::HolonomicChassis(std::initializer_list<int8_t> front_left_motors,
                         std::initializer_list<int8_t> front_right_motors,
                         std::initializer_list<int8_t> back_left_motors,
                         std::initializer_list<int8_t> back_right_motors,
                         controller_ptr default_controller, ec_ptr ec,
                         double slew_step, pros::motor_brake_mode_e brakeMode)
    : AbstractChassis(default_controller, ec) {
    this->front_left_motors = std::make_unique<pros::MotorGroup>(front_left_motors);
    this->front_right_motors = std::make_unique<pros::MotorGroup>(front_right_motors);
    this->back_left_motors = std::make_unique<pros::MotorGroup>(back_left_motors);
    this->back_right_motors = std::make_unique<pros::MotorGroup>(back_right_motors);

    this->slew_step = slew_step > 0 ? slew_step : 200;
    this->brakeMode = brakeMode;

    this->front_left_motors->set_brake_mode_all(this->brakeMode);
    this->front_right_motors->set_brake_mode_all(this->brakeMode);
    this->back_left_motors->set_brake_mode_all(this->brakeMode);
    this->back_right_motors->set_brake_mode_all(this->brakeMode);

    this->prev_voltages = {0, 0, 0};
}

void HolonomicChassis::tank(double left_speed, double right_speed) {

}

void HolonomicChassis::arcade(double forward_speed, double turn_speed) {

}

void HolonomicChassis::holonomic(double forward_speed, double sideways_speed, double turn_speed) {
    double front_left = forward_speed - sideways_speed - turn_speed;
    double front_right = forward_speed + sideways_speed + turn_speed;
    double back_left = forward_speed + sideways_speed - turn_speed;
    double back_right = forward_speed - sideways_speed + turn_speed;

    this->front_left_motors->move_voltage(120.0 * front_left);
    this->front_right_motors->move_voltage(120.0 * front_right);
    this->back_left_motors->move_voltage(120.0 * back_left);
    this->back_right_motors->move_voltage(120.0 * back_right);
}

void HolonomicChassis::set_brake_mode(pros::motor_brake_mode_e mode) {
    this->brakeMode = mode;
    this->front_left_motors->set_brake_mode_all(mode);
    this->front_right_motors->set_brake_mode_all(mode);
    this->back_left_motors->set_brake_mode_all(mode);
    this->back_right_motors->set_brake_mode_all(mode);
}

bool HolonomicChassis::execute(DiffChassisCommand cmd, double max) {
    return false;
}

// Evoke the chassis to move according to how it was set up using the
// constructor, returns true if movement is complete
bool HolonomicChassis::execute(HoloChassisCommand cmd, double max, int type) {
    return std::visit(
        overload{[this](Stop&) -> bool {
                     this->set_brake_mode(this->brakeMode);
                     this->front_left_motors->brake();
                     this->front_right_motors->brake();
                     this->back_left_motors->brake();
                     this->back_right_motors->brake();

                     return true;
                 },
                 [this, max](holo_commands::Voltages& v) -> bool {
                     double v_max = std::max(fabs(v.v_theta), std::max(fabs(v.v_x), fabs(v.v_y)));
                     if (v_max > max) {
                         v.v_x = v.v_x * max / v_max;
                         v.v_y = v.v_y * max / v_max;
                         v.v_theta = v.v_theta;
                     }

                     v.v_x = slew(v.v_x, 0);
                     v.v_y = slew(v.v_y, 1);
                     v.v_theta = slew(v.v_theta, 2);

                     this->front_left_motors->move_voltage(120 * (v.v_x - v.v_y - v.v_theta));
                     this->front_right_motors->move_voltage(120 * (v.v_x + v.v_y + v.v_theta));
                     this->back_left_motors->move_voltage(120 * (v.v_x + v.v_y - v.v_theta));
                     this->back_right_motors->move_voltage(120 * (v.v_x - v.v_y + v.v_theta));

                     this->prev_voltages = v;

                     return false;
                 },
                 // Logic allowing for individual movements within a chain of
                 // movements to be registered at completed even though robot
                 // may still be moving
                 [this, max](holo_commands::Chained& v) -> bool {
                     double v_max = std::max(fabs(v.v_theta), std::max(fabs(v.v_x), fabs(v.v_y)));
                     if (v_max > max) {
                         v.v_x = v.v_x * max / v_max;
                         v.v_y = v.v_y * max / v_max;
                         v.v_theta = v.v_theta;
                     }

                     v.v_x = slew(v.v_x, 0);
                     v.v_y = slew(v.v_y, 1);
                     v.v_theta = slew(v.v_theta, 2);

                     holonomic(v.v_x, v.v_y, v.v_theta);

                     this->prev_voltages = {v.v_x, v.v_y, v.v_theta};

                     return true;
                 },
                 // Logic to brake one side of the drive alloing for a turn
                 // around the side of the robot and returning true when the
                 // turn is finished
                 [this, max](holo_commands::Swing& v) {
                     double v_max = std::max(fabs(v.v_theta), std::max(fabs(v.v_x), fabs(v.v_y)));
                     if (v_max > max) {
                         v.v_x = v.v_x * max / v_max;
                         v.v_y = v.v_y * max / v_max;
                         v.v_theta = v.v_theta;
                     }

                     v.v_x = slew(v.v_x, 0);

                     holonomic(v.v_x, 0, v.v_x);

                    //  this->front_right_motors->set_brake_mode_all(
                    //      pros::MotorBrake::hold);
                    //  this->front_right_motors->brake();
                    //  if (v_max > max) {
                    //      v.left = v.left * max / v_max;
                    //  }
                    //  v.left = slew(v.left, true);
                    //  this->front_left_motors->move_voltage(120 * v.left);
                    //  this->prev_voltages = {v.left, 0.0};


                    //  if (v.right == 0) {
                    //      this->front_right_motors->set_brake_mode_all(
                    //          pros::MotorBrake::hold);
                    //      this->front_right_motors->brake();
                    //      if (v_max > max) {
                    //          v.left = v.left * max / v_max;
                    //      }
                    //      v.left = slew(v.left, true);
                    //      this->front_left_motors->move_voltage(120 * v.left);
                    //      this->prev_voltages = {v.left, 0.0};

                    //  } else if (v.left == 0) {
                    //      this->front_left_motors->set_brake_mode_all(
                    //          pros::MotorBrake::hold);
                    //      this->front_left_motors->brake();
                    //      if (v_max > max) {
                    //          v.right = v.right * max / v_max;
                    //      }
                    //      v.right = slew(v.right, false);

                    //      this->front_right_motors->move_voltage(120 * v.right);
                    //      this->prev_voltages = {0.0, v.right};
                    //  }

                     return false;
                 }},
        cmd);
}

} // namespace voss::chassis