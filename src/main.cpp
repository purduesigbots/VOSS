#include "main.h"
#include "VOSS/api.hpp"

#define LEFT_MOTORS                                                            \
    { -2, -3, -4 }
#define RIGHT_MOTORS                                                           \
    { 5, 6, 10 }

auto odom = voss::localizer::TrackingWheelLocalizerBuilder::new_builder()
                .with_right_motor(10)
                .with_left_motor(-4)
                .with_left_right_tpi(18.43)
                .with_imu(1)
                .build();

auto pid = voss::controller::create_controller<voss::controller::PIDController>(
    {.lin_kp = 0, .ang_kd = 10});

auto boomerang = std::shared_ptr<voss::controller::BoomerangController>({});
//
auto swing = voss::controller::SwingController::create_controller({});
//
// auto arc = voss::controller::ArcPIDController(
//               {.lin_kp = 1000, .ang_kp = 250, .track_width = 16})
//               .get_ptr();
//
//auto pp = std::make_shared<voss::controller::PPController>(50, 0, 55, 240, 0.05,
//                                                           2400, 15, 14);
auto pp = voss::controller::PPController::create_controller({});
//
#define TW 14
auto ramsete =
    voss::controller::create_controller<voss::controller::RamseteController>(
        {.zeta = 0.9,
         .b = 0.01,
         .ffwd_kV = 1.2,
         .ffwd_kS = 8,
         .ffwd_kA = 0.01,
         .ffwd_kD = 0,
         .track_width = TW,
         .gear_ratio = 36.0 / 60});

auto traj_constraints =
    voss::trajectory::TrajectoryConstraints{.max_vel = 70,
                                            .max_accel = 100,
                                            .max_decel = -50,
                                            .max_ang_accel = M_PI,
                                            .max_centr_accel = 0.0,
                                            .track_width = TW};

pros::Controller master(pros::E_CONTROLLER_MASTER);
auto ec = voss::controller::ExitConditions::new_conditions()
              .add_settle(400, 0.5, 400)
              .add_tolerance(1.0, 2.0, 200)
              .add_timeout(10000)
              .add_thru_smoothness(4)
              .build();

auto chassis = voss::chassis::DiffChassis(LEFT_MOTORS, RIGHT_MOTORS, pid, odom,
                                          ec, 0, pros::E_MOTOR_BRAKE_COAST);

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
    pros::delay(3000);
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

void display_odom() {
    while (true) {
        auto p = odom->get_position();

        pros::lcd::print(1, "%lf", p.x);
        pros::lcd::print(2, "%lf", p.y);
        pros::lcd::print(3, "%lf", odom->get_orientation_deg());
        pros::delay(10);
    }
}

void autonomous() {
    odom->set_pose({0.0, 0.0, 90});
    chassis.follow_path({{0, 0}, {0, 48}, {48, 60, 45}}, pp, 80);
    printf("exit\n");
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
    pros::Task tk(display_odom);
    while (true) {

        chassis.arcade(master.get_analog(ANALOG_LEFT_Y),
                       master.get_analog(ANALOG_RIGHT_X));

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            autonomous();
        }
        pros::delay(10);
    }
}