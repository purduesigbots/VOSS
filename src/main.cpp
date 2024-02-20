#include "main.h"
#include "VOSS/api.hpp"
#include "VOSS/controller/BoomerangControllerBuilder.hpp"
#include "VOSS/controller/PIDControllerBuilder.hpp"
#include "VOSS/controller/SwingControllerBuilder.hpp"
#include "VOSS/localizer/ADILocalizerBuilder.hpp"
#include "VOSS/utils/flags.hpp"

#define LEFT_MOTORS                                                            \
    { 11, -12, -1, 2, -16 }
#define RIGHT_MOTORS                                                           \
    { -20, 19, 6, 8, -18 }

auto odom = voss::localizer::IMELocalizerBuilder::new_builder()
                .with_left_motors(LEFT_MOTORS)
                .with_right_motors(RIGHT_MOTORS)
                .with_left_right_tpi(17)
                .with_imu(10)
                .build();

auto pid = voss::controller::PIDControllerBuilder::new_builder(odom)
               .with_linear_constants(5, 0, 12)
               .with_angular_constants(170, 0.05, 700)
               .with_tracking_kp(60)
               .with_exit_error(1)
               .with_angular_exit_error(2)
               .with_min_error(5)
               .with_settle_time(200)
               .build();

auto chassis = voss::chassis::DiffChassis(LEFT_MOTORS, RIGHT_MOTORS, pid, 8);

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
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    auto odom = voss::localizer::IMELocalizerBuilder::new_builder()
                    .with_left_motors({-2, -3, -12, 1, 11})
                    .with_right_motors({-7, -20, 6, 5, 19})
                    .with_left_right_tpi(17.5) // 19.5
                    .with_track_width(11)      // 3.558
                    .with_imu(13)
                    .build();

    odom->begin_localization();

    auto pid = voss::controller::PIDControllerBuilder::new_builder(odom)
                   .with_linear_constants(10, 0.01, 52)
                   .with_angular_constants(240, 0.05, 2400)
                   .with_exit_error(1.0)
                   .with_angular_exit_error(2.0)
                   .with_min_error(5)
                   .with_settle_time(200)
                   .build();

    auto boomerang =
        voss::controller::BoomerangControllerBuilder::new_builder(odom)
            .with_linear_constants(10, 0.01, 52)
            .with_angular_constants(240, 0.05, 2400)
            .with_exit_error(1.0)
            .with_lead_pct(0.6)
            .with_angular_exit_error(1.0)
            .with_min_error(5)
            .with_settle_time(200)
            .with_min_vel_for_thru(50)
            .build();

    auto swing = voss::controller::SwingControllerBuilder::new_builder(odom)
                     .with_angular_constants(170, 0, 700)
                     .with_angular_exit_error(0.5)
                     .with_settle_time(200)
                     .build();

    voss::chassis::DiffChassis chassis({-2, -3, -12, 1, 11},
                                       {-7, -20, 6, 5, 19}, pid, 8);

    auto [leftM, rightM] = chassis.getMotors();

    while (true) {
        voss::Pose p = odom->get_pose();

        chassis.arcade(master.get_analog(ANALOG_LEFT_Y),
                       master.get_analog(ANALOG_RIGHT_X));

        if (master.get_digital_new_press(DIGITAL_Y)) {
            odom->set_pose(voss::Pose{0.0, 0.0, 180});
            //            chassis.move(voss::Pose{24, 24, 90}, &boomerang,
            //            100.0, voss::Flags::THRU); master.rumble(".");
            //            chassis.move(voss::Pose{48, 5, 290}, &boomerang, 70.0,
            //            voss::Flags::THRU); master.rumble(".");
            //            chassis.move(voss::Pose{64, 24, 90},
            //            &boomerang, 70.0); master.rumble(".");
            chassis.move(voss::Pose{48, 25, 10}, boomerang, 70.0,
                         voss::Flags::REVERSE | voss::Flags::THRU);
            //            chassis.move(voss::Pose{48, 25, 10}, &boomerang, 70.0,
            //            voss::Flags::THRU);
            master.rumble(".");
            chassis.move(voss::Pose{60, -5, 270}, boomerang, 100.0,
                         voss::Flags::REVERSE);
            //            chassis.move(voss::Pose{60, 0, 270},
            //            &boomerang, 70.0);
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