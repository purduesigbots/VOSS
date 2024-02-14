#include "main.h"
#include "VOSS/api.hpp"
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
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
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    auto odom = voss::localizer::IMELocalizerBuilder::new_builder()
                    .with_left_motors({-2, -3, -6, -5})
                    .with_right_motors({11, 12, 19, 20})
                    .with_left_right_tpi(18.24) // 19.5
                    .with_track_width(9.75)     // 3.558
                    .with_imu(13)
                    .build();

    odom->begin_localization();

    auto pid = voss::controller::PIDControllerBuilder::new_builder(odom)
                   .with_linear_constants(7, 0.02, 40)
                   .with_angular_constants(170, 0, 700)
                   .with_exit_error(1.0)
                   .with_angular_exit_error(1.0)
                   .with_min_error(5)
                   .with_settle_time(200)
                   .build();
    auto arc = voss::controller::ArcPIDControllerBuilder::new_builder(odom)
                   .with_linear_constants(6, 0, 50)
                   .with_track_width(14)
                   .with_exit_error(1.0)
                   .with_min_error(5.0)
                   .with_settle_time(200)
                   .with_slew(8)
                   .build();

    auto swing = voss::controller::SwingControllerBuilder::new_builder(odom)
                     .with_angular_constants(170, 0, 700)
                     .with_angular_exit_error(0.5)
                     .with_settle_time(200)
                     .build();

    voss::chassis::DiffChassis chassis({-2, -3, -6, -5}, {11, 12, 19, 20}, pid,
                                       8);

    auto [leftM, rightM] = chassis.getMotors();

    while (true) {

        voss::Pose p = odom->get_pose();

        chassis.arcade(master.get_analog(ANALOG_LEFT_Y),
                       master.get_analog(ANALOG_RIGHT_X));

        if (master.get_digital_new_press(DIGITAL_Y)) {
            odom->set_pose(voss::Pose{0.0, 0.0, 0});
            chassis.turn(270, 100, voss::Flags::NONE,
                         voss::AngularDirection::COUNTERCLOCKWISE);
            chassis.turn(0);
            chassis.turn(-270, swing, 100, voss::Flags::NONE,
                         voss::AngularDirection::CLOCKWISE);
        }

        pros::lcd::clear_line(1);
        pros::lcd::clear_line(2);
        pros::lcd::clear_line(3);
        pros::lcd::print(1, "%lf", p.x);
        pros::lcd::print(2, "%lf", p.y);
        pros::lcd::print(3, "%lf", odom->get_orientation_deg());
        pros::lcd::print(4, "%s", (odom == nullptr) ? "true" : "false");
        pros::delay(10);
    }
}