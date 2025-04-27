#include "main.h"
#include "Eigen/Dense"
#include "VOSS/api.hpp"
#include "VOSS/controller/BoomerangControllerBuilder.hpp"
#include "VOSS/controller/PIDControllerBuilder.hpp"
#include "VOSS/controller/SwingControllerBuilder.hpp"
#include "VOSS/localizer/ADILocalizerBuilder.hpp"
#include "VOSS/utils/flags.hpp"

#define VERTICAL_TPI (159.880702835)
#define HORIZONTAL_TPI (316.40129134)

#define VERTICAL_DIST (1.375)
#define HORIZONTAL_DIST (2.15625)

#define GPS_PORT (8)
#define IMU_PORTS                                                              \
    { 12 }

#define LEFT_MOTORS                                                            \
    { -1, 3, -4, -5, 6 }
#define RIGHT_MOTORS                                                           \
    { 20, -19, 18, -17, 16 }

#define ADI_EXPANDER_PORT (11)
#define VERTICAL_TRACKER_PORT ('A')   // and 'B'
#define HORIZONTAL_TRACKER_PORT ('C') // and 'D'

auto odom = voss::localizer::TrackingWheelLocalizerBuilder::new_builder()
                .with_left_encoder(ADI_EXPANDER_PORT, VERTICAL_TRACKER_PORT)
                .with_middle_encoder(ADI_EXPANDER_PORT, HORIZONTAL_TRACKER_PORT)
                .with_left_right_tpi(VERTICAL_TPI)
                .with_middle_tpi(HORIZONTAL_TPI)
                .with_track_width(VERTICAL_DIST * 2)
                .with_middle_dist(HORIZONTAL_DIST)
                .with_imus(IMU_PORTS)
                .build();

/*
auto pure_odom =
    voss::localizer::TrackingWheelLocalizerBuilder::new_builder()
        .with_left_encoder(ADI_EXPANDER_PORT, VERTICAL_TRACKER_PORT)
        .with_middle_encoder(ADI_EXPANDER_PORT, HORIZONTAL_TRACKER_PORT)
        .with_left_right_tpi(VERTICAL_TPI)
        .with_middle_tpi(HORIZONTAL_TPI)
        .with_track_width(VERTICAL_DIST * 2)
        .with_middle_dist(HORIZONTAL_DIST)
        .with_imus(IMU_PORTS)
        .build();
*/
//
// std::shared_ptr<voss::localizer::TrackingWheelLocalizer> odom =
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

// std::shared_ptr<voss::localizer::TrackingWheelLocalizer> odom2 =
//     voss::localizer::TrackingWheelLocalizerBuilder::new_builder()
//         .with_middle_encoder(MIDDLE_TRACKER_PORT_2)
//         .with_left_encoder(LEFT_TRACKER_PORT_2)
//         .with_middle_tpi(300.0 * 13.4 / 12.0 * 11.95 / 12.0 * 21.7 / 24.0 *
//                          26.2 / 24.0)
//         .with_left_right_tpi(300.0 * 13.4 / 12.0 * 11.95 / 12.0)
//         .with_track_width(2.375 * 2.0)
//         .with_middle_dist(4.625)
//         .with_imus({7, 10})
//         .build();

auto pid = voss::controller::PIDControllerBuilder::new_builder(odom)
               .with_linear_constants(20, 0.02, 169)
               .with_angular_constants(250, 0.05, 2435)
               .with_min_error(5)
               .with_min_vel_for_thru(100)
               .build();
//
// auto boomerang =
// voss::controller::BoomerangControllerBuilder::new_builder(odom)
//                      .with_linear_constants(20, 0.02, 169)
//                      .with_angular_constants(250, 0.05, 2435)
//                      .with_lead_pct(0.5)
//                      .with_min_vel_for_thru(70)
//                      .with_min_error(5)
//                      .build();
//
// auto swing = voss::controller::SwingControllerBuilder::new_builder(odom)
//                  .with_angular_constants(250, 0.05, 2435)
//                  .build();
//
// auto arc = voss::controller::ArcPIDControllerBuilder(odom)
//                .with_track_width(16)
//                .with_linear_constants(20, 0.02, 169)
//                .with_angular_constants(250, 0.05, 2435)
//                .with_min_error(5)
//                .with_slew(8)
//                .build();

pros::Imu imu(12);
pros::Controller master(pros::E_CONTROLLER_MASTER);
auto ec = voss::controller::ExitConditions::new_conditions()
              .add_settle(400, 0.5, 400)
              .add_tolerance(1.0, 2.0, 200)
              .add_timeout(22500)
              .add_thru_smoothness(4)
              .build()
              ->exit_if([]() {
                  return master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
              });

auto chassis = voss::chassis::DiffChassis(LEFT_MOTORS, RIGHT_MOTORS, pid, ec, 8,
                                          pros::E_MOTOR_BRAKE_COAST);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    odom->gps = std::make_shared<pros::GPS>(GPS_PORT);
    odom->kalman_filter =
        std::make_shared<voss::EKFilter>(0.01, 0.01, 0.001, 1);
    odom->begin_localization();
    //    pure_odom->begin_localization();
    pros::delay(500);
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
    pros::screen::touch_callback([] { odom->set_pose(0, 0, 0); },
                                 pros::E_TOUCH_RELEASED);
    auto gps_s = odom->gps->get_position_and_orientation();

    voss::Pose gps_pose(gps_s.x * meter_to_inch, gps_s.y * meter_to_inch,
                        voss::norm_deg(gps_s.yaw * -1.f));
    odom->set_pose(gps_pose);

    pros::delay(800);

    while (true) {
        chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                       master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        auto ekf_odom_p = odom->get_pose();
        //        auto p_odom_p = pure_odom->get_pose();
        gps_s = odom->gps->get_position_and_orientation();
        auto P = odom->kalman_filter->get_covariance();

        gps_pose =
            voss::Pose{gps_s.x * meter_to_inch, gps_s.y * meter_to_inch,
                       voss::to_radians(voss::norm_deg(gps_s.yaw * -1.f))};

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            std::cout << std::format("ekf: {:.2f}, {:.2f}, {:.2f}",
                                     ekf_odom_p.x, ekf_odom_p.y,
                                     ekf_odom_p.theta.value())
                      << std::endl;

            //        std::cout << std::format("pure: {:.2f}, {:.2f}, {:.2f}",
            //        p_odom_p.x,
            //                                 p_odom_p.y,
            //                                 p_odom_p.theta.value())
            //                  << std::endl;

            std::cout << std::format("gps: {:.2f}, {:.2f}, {:.2f}", gps_pose.x,
                                     gps_pose.y, gps_pose.theta.value())
                      << std::endl;
        }

        //
        //        std::cout << std::format("gps_error: {:.2f}",
        //                                 odom->gps->get_error() *
        //                                 meter_to_inch)
        //                  << std::endl;

        //        std::cout << std::format("imu: {:.2f}", -imu.get_rotation())
        //        << std::endl;

        //                std::cout << std::format("covariance: {:.2f}, {:.2f},
        //                {:.2f}",
        //                                          P(0, 0), P(1, 1), P(2, 2))
        //                          << std::endl;

        pros::delay(10);
    }
}