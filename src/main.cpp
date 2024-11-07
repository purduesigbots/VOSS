#include "main.h"

//#include "SSOV/chassis/DiffChassis.hpp"
#include "RobotOdom.h"
#include "SSOV/controller/PIDPointController.hpp"
#include "SSOV/exit_condition/ToleranceExitCondition.hpp"
#include "SSOV/Routines.hpp"

//ssov::DiffChassis chassis({-13, -14, -15}, {16, 18, 19});
//RobotOdom odom({-13, -14, -15}, {16, 18, 19}, 12, 20);
//ssov::PIDPointController controller({0, 0, 0}, {0, 0, 0}, 0);
auto chassis = ssov::DiffChassis::create({-13, -14, -15}, {16, 18, 19});
auto odom = std::make_shared<RobotOdom>(std::initializer_list<int8_t>{-13, -14, -15}, std::initializer_list<int8_t>{16, 18, 19}, 12, 20);
auto pid = std::make_shared<ssov::PIDPointController>(ssov::PIDConstants{20, 2, 1.69}, ssov::PIDConstants{250, 5, 24.35}, 5);
auto ec = std::make_shared<ssov::ToleranceExitCondition>(1.0, 0.04, 200);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	ssov::defaults::chassis = chassis;
	ssov::defaults::localizer = odom;
	ssov::defaults::point_controller = pid;
	ssov::defaults::exit_condition = ec;
	odom->calibrate();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
void autonomous() {}

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
	bool log_data = false;

	while (true) {
		ssov::Pose pose = odom->get_pose();
		pros::lcd::print(1, "%.2f %.2f %.2f", pose.x, pose.y, pose.theta);

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		chassis->arcade(dir / 1.27, turn / 1.27);
		//if (master.get_digital_new_press(DIGITAL_A)) {
		//	odom->set_pose({0, 0, 0});
		//	ssov::move({-24, 0}, {.reverse = true});
		//}
		if (master.get_digital_new_press(DIGITAL_X)) {
			log_data = !log_data;
		}
		auto local_change = odom->get_local_change();
		auto vel = odom->get_velocities();
		if (log_data) {
			//printf("%.2f %.2f %.2f\n", odom->get_left_velocity(), odom->get_right_velocity(), odom->get_rot_velocity());
			//printf("%.2f %.2f %.2f %.2f %.2f %.2f\n", local_change.x * 100, local_change.y * 100, local_change.theta * 100, vel.x, vel.y, vel.theta);
			printf("%.2f, %.2f\n", vel.y, local_change.y * 100);
		}
		pros::delay(10);                               // Run for 20 ms then update
	}
}