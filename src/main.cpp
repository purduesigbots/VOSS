#include "main.h"
#include "VOSS/utils/flags.hpp"
#include "pros/llemu.hpp"
#include "voss/api.hpp"

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

	auto odom = voss::localizer::IMELocalizerBuilder::newBuilder()
	                .withleftMotors({-13, -15, -16})
	                .withrightMotors({8, 7, 5})
	                .withLeftRightTPI(0)//19.5
	                .withMiddleTPI(325)
	                .withTrackWidth(3.558)
	                .build();

	//odom->begin_localization();

	//auto pid = voss::controller::PIDControllerBuilder::newBuilder(*odom)
	//               .withLinearConstants(7, 0.02, 40)
	//               .withAngularConstants(3, 0.03, 35)
	//               .withExitError(1.0)
	//               .withMinError(5)
	//               .build();
//
	//voss::chassis::DiffChassis chassis({-13, -15, -16}, {8, 7, 5}, pid);

	while (true) {
		//chassis.arcade(master.get_analog(ANALOG_LEFT_Y) * 128.0 / 100.0,
		               //master.get_analog(ANALOG_RIGHT_X) * 128.0 / 100.0);

		voss::Pose p = odom->get_pose();

		if (master.get_digital_new_press(DIGITAL_Y)) {
			odom->set_pose(voss::Pose{0.0, 0.0, 0.0});

			//chassis.move(voss::Point{-24.0, 0.0}, 100.0, voss::REVERSE);
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
