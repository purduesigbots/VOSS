#include "main.h"
#include "VOSS/api.hpp"
#include "VOSS/controller/BoomerangControllerBuilder.hpp"
#include "VOSS/controller/PIDControllerBuilder.hpp"
#include "VOSS/controller/SwingControllerBuilder.hpp"
#include "VOSS/localizer/ADILocalizerBuilder.hpp"
#include "VOSS/utils/flags.hpp"
#include "Eigen/Dense"


#define MIDDLE_TRACKER_PORT ('E')
#define LEFT_TRACKER_PORT ('C')
#define RIGHT_TRACKER_PORT ('A')

#define MIDDLE_TRACKER_PORT_2 ('G')
#define LEFT_TRACKER_PORT_2 ('A')
//
//std::shared_ptr<voss::localizer::TrackingWheelLocalizer> odom =
//    voss::localizer::TrackingWheelLocalizerBuilder::new_builder()
//        .with_middle_encoder(MIDDLE_TRACKER_PORT)
//        .with_left_encoder(LEFT_TRACKER_PORT)
//        .with_right_encoder(RIGHT_TRACKER_PORT)
//        .with_middle_tpi(5000 / 24.0)
//        .with_left_tpi(4988 / 24.0)
//        .with_right_tpi(4988 / 24.0)
//        .with_track_width(4.0625)
//        .with_middle_dist(-4.625)
//        .build();

std::shared_ptr<voss::localizer::TrackingWheelLocalizer> odom2 =
    voss::localizer::TrackingWheelLocalizerBuilder::new_builder()
        .with_middle_encoder(MIDDLE_TRACKER_PORT_2)
        .with_left_encoder(LEFT_TRACKER_PORT_2)
        .with_middle_tpi(300.0 * 13.4 / 12.0 * 11.95 / 12.0 * 21.7 / 24.0 *
                         26.2 / 24.0)
        .with_left_right_tpi(300.0 * 13.4 / 12.0 * 11.95 / 12.0)
        .with_track_width(2.375 * 2.0)
        .with_middle_dist(4.625)
        .with_imus({7, 10})
        .build();

//auto pid = voss::controller::PIDControllerBuilder::new_builder(odom)
//               .with_linear_constants(20, 0.02, 169)
//               .with_angular_constants(250, 0.05, 2435)
//               .with_min_error(5)s
//               .with_min_vel_for_thru(100)
//               .build();
//
//auto boomerang = voss::controller::BoomerangControllerBuilder::new_builder(odom)
//                     .with_linear_constants(20, 0.02, 169)
//                     .with_angular_constants(250, 0.05, 2435)
//                     .with_lead_pct(0.5)
//                     .with_min_vel_for_thru(70)
//                     .with_min_error(5)
//                     .build();
//
//auto swing = voss::controller::SwingControllerBuilder::new_builder(odom)
//                 .with_angular_constants(250, 0.05, 2435)
//                 .build();
//
//auto arc = voss::controller::ArcPIDControllerBuilder(odom)
//               .with_track_width(16)
//               .with_linear_constants(20, 0.02, 169)
//               .with_angular_constants(250, 0.05, 2435)
//               .with_min_error(5)
//               .with_slew(8)
//               .build();

pros::Controller master(pros::E_CONTROLLER_MASTER);
//auto ec = voss::controller::ExitConditions::new_conditions()
//              .add_settle(400, 0.5, 400)
//              .add_tolerance(1.0, 2.0, 200)
//              .add_timeout(22500)
//              .add_thru_smoothness(4)
//              .build()
//              ->exit_if([]() {
//                  return master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
//              });

//auto chassis = voss::chassis::DiffChassis(LEFT_MOTORS, RIGHT_MOTORS, pid, ec, 8,
//                                          pros::E_MOTOR_BRAKE_COAST);

pros::IMU imu(16);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    odom2->begin_localization();
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
    pros::screen::touch_callback([] { odom2->set_pose(0, 0, 0); },
                                 pros::E_TOUCH_RELEASED);



    while (true) {
        voss::Pose pp = odom2->get_pose();
        std::cout << std::format("{:.2f},{:.2f},{:.2f}", pp.x,
                                 pp.y, pp.theta.value())
                  << std::endl;
        pros::delay(10);
    }
}