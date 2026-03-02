#include "main.h"

#include "SSOV/chassis/DiffChassis.hpp"
#include "SSOV/chassis/HoloChassis.hpp"

#include "SSOV/trajectory/PathTrajectory.hpp"
#include "SSOV/trajectory/QuinticSplinePath.hpp"
#include "SSOV/controller/RamseteTrajectoryFollower.hpp"
#include "SSOV/trajectory/CombinedTrajectory.hpp"
#include "SSOV/controller/PIDPointController.hpp"
#include "SSOV/controller/PIDPoseController.hpp"
#include "SSOV/controller/PIDTurnController.hpp"
#include "SSOV/exit_condition/ToleranceExitCondition.hpp"
#include "SSOV/exit_condition/TimeoutExitCondition.hpp"

#include "SSOV/localizer/TrackingWheelLocalizer.hpp"
#include "SSOV/localizer/ADITrackingWheel.hpp"
#include "SSOV/localizer/HoloRobotOdom.h"
#include "SSOV/localizer/RobotOdom.h"

#include "replay/replay.hpp"

//Tracker wheels
//std::unique_ptr<ssov::AbstractTrackingWheel> left = std::make_unique<ssov::ADITrackingWheel>('e', 310.7);
std::unique_ptr<ssov::AbstractTrackingWheel> right = std::make_unique<ssov::ADITrackingWheel>('e', 155.35); //310.7*3.5
std::unique_ptr<ssov::AbstractTrackingWheel> middle = std::make_unique<ssov::ADITrackingWheel>('g', 153.5); //310.7*3.5
//----------------------------------------------------------------------------------------------------------
//std::move(middle)
auto imuOdom = std::make_shared<HoloRobotOdom>(std::initializer_list<int8_t>{10,-9}, std::initializer_list<int8_t>{5,-6}, std::initializer_list<int8_t>{7,-8}, std::initializer_list<int8_t>{4,-3}, 20);
auto imu = std::make_unique<pros::IMU>(1);
auto odom = std::make_shared<ssov::TrackingWheelLocalizer>(nullptr, std::move(right), std::move(middle), std::move(imu), 3.75, -1.5, ssov::Pose{0, 0, 0});
auto chassis = ssov::HolonomicChassis::create({10,-9}, {5,-6}, {7,-8}, {4,-3});
auto pid = std::make_shared<ssov::PIDPointController>(ssov::PIDConstants{20, 2, 1.69}, ssov::PIDConstants{2, 0, 0}, 5);
auto ec = std::make_shared<ssov::ToleranceExitCondition>(2, 2, 400);
auto ec_time = std::make_shared<ssov::TimeoutExitCondition>(10000);
auto ec_thru = std::make_shared<ssov::ToleranceExitCondition>(6, 1, 200);
auto pid_pose = std::make_shared<ssov::PIDPoseController>(ssov::PIDConstants{10, 0, 2}, ssov::PIDConstants{150, 0, 2}, 1);
auto turn_pid = std::make_shared<ssov::PIDTurnController>(ssov::PIDConstants{150, 0, 2}, 1);

// auto odom = std::make_shared<ssov::TrackingWheelLocalizer>(std::move(left), nullptr, std::move(middle), std::move(imu), 0, 0, ssov::Pose{-2.125, 0, -M_PI_4});
// auto ramsete = std::make_shared<ssov::RamseteTrajectoryFollower>(0.00258064, 0.7, 1.47410043, 8.3411535, 2.09563917, 14.6568819);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	chassis->default_point_controller = pid;
	pid_pose->final_angle_distance = 0;
	pid_pose->sideways_multiplier = 2;
	chassis->default_pose_controller = pid_pose;
	chassis->default_ec = ec_time;
	chassis->default_turn_controller = turn_pid;
	odom->imu_dir = -1;
	odom->begin_localization();
	chassis->register_localizer(odom);
	odom->set_pose({0, 0, 0});
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
void autonomous() {
	odom->set_pose({0, 0, 0});
	
	//chassis->default_ec = ec_thru;
	std::cout << "Running auto"<< std::endl;
	chassis->turn(45);

	// pros::delay(20000);

	// chassis->move({24, 0, 0}, 0, {.max=25, .thru=false, .holonomic = true});

	// chassis->move({24, 15, 0}, 0, {.max=50, .thru=false, .holonomic = true});
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
void print_odom_val(){
		ssov::Pose pose = odom->get_pose();
		ssov::DriveSignal current_signal = chassis->get_current_drive_signal();
		while(true){
			pose = odom->get_pose();
			current_signal = chassis->get_current_drive_signal();
			std::cout << "X: " << pose.x << ", Y: " << pose.y << ", Theta: " << pose.theta << std::endl;
			std::cout << "X power: " << current_signal.x << ", Y power: " << current_signal.y << ", Theta power: " << current_signal.theta << std::endl;
			pros::delay(200);
		}
		
	}

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	ssov::Pose pose = odom->get_pose();
	bool log_data = false;
	//pros::Task print_odom(print_odom_val);

	int timer = 0;
	while (true) {
		pose = odom->get_pose();
		pros::lcd::print(1, "%.2f %.2f %.2f", pose.x, pose.y, pose.theta);
		//replay::Packet packet;
		//packet.add_pose("robot location", pose.x, pose.y, pose.theta);
		//logger.log(packet);

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		int strafe = master.get_analog(ANALOG_LEFT_X);
		chassis->arcade(dir / 1.27, turn / 1.27, strafe / 1.27);

		// Diff drive tank control----------------------------------------
		//int left = master.get_analog(ANALOG_LEFT_Y);
		//int right = master.get_analog(ANALOG_RIGHT_Y);
		//chassis->tank(left / 1.27, right / 1.27);
		//----------------------------------------------------------------

		if (master.get_digital_new_press(DIGITAL_X)) {
			log_data = !log_data;
		}
		if(master.get_digital_new_press(DIGITAL_A)) {
			//FILE *file = fopen("/usd/ff.txt", "w");
			autonomous();
			//for (double i = 0.0; i <= traj.duration(); i += 0.01) {
				//auto vel = odom->get_velocities();
				//auto pose = odom->get_pose();
				//auto state = traj.at(i);
				//auto speeds = ramsete->compute(pose, vel, state);
				//chassis->execute(speeds);
				//fprintf(file, "%.2f, %.2f, %.2f, %.2f, %.2f\n", speeds.left_speed * 0.12, speeds.right_speed * 0.12, vel.x, vel.y, vel.theta);
				//fprintf(file, "%f, %f, %f, %f, %f\n", state.vel.x, state.vel.theta, vel.x, vel.y, vel.theta);
				//pros::delay(10);
			//}
			//fclose(file);
		}
		//auto local_change = odom->get_local_change();
		auto vel = odom->get_velocities();
		// if (log_data) {
		// 	// auto speeds = chassis->get_speeds();
		// 	//printf("%.2f %.2f %.2f\n", odom->get_left_velocity(), odom->get_right_velocity(), odom->get_rot_velocity());
		// 	//printf("%.2f %.2f %.2f %.2f %.2f %.2f\n", local_change.x * 100, local_change.y * 100, local_change.theta * 100, vel.x, vel.y, vel.theta);
		// 	//fprintf(file, "%.2f, %.2f, %.2f, %.2f, %.2f\n", speeds.left_speed * 0.12, speeds.right_speed * 0.12, vel.x, vel.y, vel.theta);
		// }

		// if (timer > 50){
		// 	std::cout << "X: " << pose.x << ", Y: " << pose.y << ", Theta: " << pose.theta << std::endl;
		// 	timer = 0;
		// }
		// timer++;
		
		pros::delay(10);
	}
}