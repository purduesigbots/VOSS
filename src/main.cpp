#include "main.h"
#include "VOSS/api.hpp"
#include "VOSS/controller/BoomerangControllerBuilder.hpp"
#include "VOSS/controller/PIDControllerBuilder.hpp"
#include "VOSS/controller/SwingControllerBuilder.hpp"
#include "VOSS/localizer/ADILocalizerBuilder.hpp"
#include "VOSS/utils/flags.hpp"
#include <iostream>
//
//#define LEFT_MOTORS                                                            \
//    { -4, -1, -21, 8, 13 }
//#define RIGHT_MOTORS                                                           \
//    { 10, 3, 9, -7, -15 }

auto odom = voss::localizer::TrackingWheelLocalizerBuilder::new_builder().build();



auto pid = voss::controller::PIDControllerBuilder::new_builder(odom).build();



auto ec = voss::controller::ExitConditions::new_conditions().build();

auto chassis = voss::chassis::DiffChassis({}, {}, pid, ec);

pros::Motor intake(5);

pros::adi::Pneumatics piston1('A', false);

//auto chassis = voss::chassis::DiffChassis(LEFT_MOTORS, RIGHT_MOTORS, pid, ec, 8,
//                                          pros::E_MOTOR_BRAKE_COAST);

bool piston_state = false;
void piston_toggle() {
    piston_state = !piston_state;
    piston1.set_value(piston_state);
}



pros::IMU imu(16);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    odom->begin_localization();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    // auto odom = voss::localizer::ADILocalizerBuilder::new_builder().build();
    // auto pid =
    // voss::controller::BoomerangControllerBuilder::new_builder(odom)
    //                .with_lead_pct(60)
    //                .build();

    // // auto pid2 =
    // // voss::controller::BoomerangControllerBuilder::new_builder(odom)
    // //                 .with_lead_pct(65)
    // //                 .build();

    // auto pid2 = voss::controller::ControllerCopy(pid).modify_lead_pct(65);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void print_time() {
    while(true) {
        pros::lcd::print(5, "%d", pros::millis());

        pros::delay(10);
    }
}



void opcontrol() {
    pros::Task timer(print_time);





    pros::Controller controller1(pros::E_CONTROLLER_MASTER);

    while (true) {

        controller1.set_text(0, 0, "hello driver");
        if(controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            controller1.rumble("-...-");
        }

        chassis.tank(controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                     controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        if(controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move_voltage(12000);
        } else if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            intake.move_voltage(-12000);
        } else {
            intake.brake();
        }

        if(controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            piston_toggle();
        }

        pros::lcd::print(0, "Piston1: %s", piston_state ? "ON" : "OFF");
        pros::lcd::print(2, "Motor: %f", intake.get_position());

        printf("IMU: %f\n", imu.get_rotation());


        pros::delay(10);//10 ms = 0.01 s
    }
}