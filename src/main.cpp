#include "main.h"
#include "VOSS/localizer/ADILocalizerBuilder.hpp"
#include "voss/api.hpp"
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();

	int valid = 0;
	int invalid = 0;

	for (int i = 0; i < 256; i++) {
		std::vector<int8_t> left = {};
		std::vector<int8_t> right = {};
		std::vector<int8_t> horiz = {};
		double lr_tpi = 0.0;
		double mid_tpi = 0.0;
		double track_width = 0.0;
		double mid_dist = 0.0;

		if (i & 0b10000000) {
			left = {1};
		}
		if (i & 0b01000000) {
			right = {2};
		}
		if (i & 0b00100000) {
			lr_tpi = 1.0;
		}
		if (i & 0b00010000) {
			track_width = 1.0;
		}
		if (i & 0b00001000) {
			horiz = {3};
		}
		if (i & 0b00000100) {
			mid_tpi = 1.0;
		}
		if (i & 0b00000010) {
			mid_dist = 1.0;
		}

		auto odom = voss::localizer::IMELocalizerBuilder::newBuilder()
		                .withleftMotors(left)
		                .withrightMotors(right)
		                .withhorizontalMotors(horiz)
		                .withLeftRightTPI(lr_tpi)
		                .withMiddleTPI(mid_tpi)
		                .withTrackWidth(track_width)
		                .withMiddleDistance(mid_dist)
		                .build();

		if (odom == nullptr) {
			invalid++;
		} else {
			valid++;
			printf("Valid %d: Case %d\n", valid, i);
		}

		pros::delay(10);
	}

	printf("# Valid: %d\n", valid);
	printf("# Invalid: %d\n", invalid);
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
	                 .withLeftRightTPI(19.5) // 19.5
	                 .withMiddleTPI(325)
	                 .withTrackWidth(8.4) //3.558
					 //.withIMU(18)
	                 .build();
//
	//auto odom = voss::localizer::ADILocalizerBuilder::newBuilder()
	//                 .withLeftEncoder(-1)
	//                 .withRightEncoder(3)
	//                 .withLeftRightTPI(325) // 19.5 //325
	//                 .withMiddleTPI(325)
	//                 .withTrackWidth(3.558) //3.558
	//				 .withIMU(18)
	//                 .build();
//
	 odom->begin_localization();

	auto pid = voss::controller::PIDControllerBuilder::newBuilder(odom)
	               .withLinearConstants(7, 0.02, 40)
	               .withAngularConstants(170, 0, 700)
	               .withExitError(1.0)
	               .withAngularExitError(1.0)
	               .withMinError(5)
	               .withSettleTime(200)
	               .build();

	voss::chassis::DiffChassis chassis({-13, -15, -16}, {8, 7, 5}, pid, 8);

	while (true) {

		 voss::Pose p = odom->get_pose();

		if (master.get_digital_new_press(DIGITAL_Y)) {
			odom->set_pose(voss::Pose{0.0, 0.0, 0.0});
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