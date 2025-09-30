#include "main.h"
#include "VOSS/api.hpp"
#include "VOSS/controller/BoomerangControllerBuilder.hpp"
#include "VOSS/controller/PIDControllerBuilder.hpp"
#include "VOSS/controller/SwingControllerBuilder.hpp"
#include "VOSS/localizer/ADILocalizerBuilder.hpp"
#include "VOSS/utils/flags.hpp"

#define LEFT_FRONT                                                            \
    { 10, -9 }
#define RIGHT_FRONT                                                           \
    { 7, -8 }
#define LEFT_BACK                                                           \
    { 5, -6 }
#define RIGHT_BACK                                                           \
    { 4, -3 }

pros::Motor Back_Intake (-19);
pros::Motor Front_Intake (-2);
pros::Motor Middle_Intake (-11);
pros::Motor Back_Right (-17);
pros::Motor Back_Left (18);
pros::Optical Front_optical (16);
pros::Optical Back_optical (1);
pros::adi::Pneumatics extendoIntake ('a', false);
pros::adi::Pneumatics rearIntake ('c', false);
pros::adi::Pneumatics colorSort ('b', false);
pros::adi::Pneumatics aligner ('d', false);

auto odom = voss::localizer::TrackingWheelLocalizerBuilder::new_builder()
                .with_left_encoder(3)
                .with_middle_encoder(1)
                .with_imu(19)
                //.with_left_right_tpi(522)
                //.with_middle_tpi(522)
                .with_track_width(2)
                .with_middle_dist(1.5)
                .build();

auto pid = voss::controller::PIDControllerBuilder::new_builder(odom)
               .with_linear_constants(8, 0, 70)
               .with_angular_constants(250, 0.001, 2500)
               .with_min_error(5)
               .with_min_vel_for_thru(40)
               .build();

auto boomerang = voss::controller::BoomerangControllerBuilder::new_builder(odom)
                     .with_linear_constants(8, 0, 70)
                     .with_angular_constants(250, 0.001, 2500)
                     .with_lead_pct(0.6)
                     .with_min_vel_for_thru(70)
                     .with_min_error(10)
                     .build();

auto swing = voss::controller::SwingControllerBuilder::new_builder(odom)
                 .with_angular_constants(250, 0.05, 2435)
                 .build();

auto arc = voss::controller::ArcPIDControllerBuilder(odom)
               .with_track_width(16)
               .with_linear_constants(20, 0.02, 169)
               .with_angular_constants(250, 0.05, 2435)
               .with_min_error(5)
               .with_slew(8)
               .build();

pros::Controller master(pros::E_CONTROLLER_MASTER);
auto ec = voss::controller::ExitConditions::new_conditions()
              .add_settle(400, 0.5, 400)
              .add_tolerance(1.0, 2.0, 200)
              .add_timeout(22500)
              .add_thru_smoothness(4)
              .build() -> exit_if([]() {
                  return master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
              });

auto chassis = voss::chassis::HolonomicChassis(LEFT_FRONT, RIGHT_FRONT, LEFT_BACK, RIGHT_BACK, pid, ec, 8,
                                          pros::E_MOTOR_BRAKE_COAST);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    odom->begin_localization();

    pros::lcd::register_btn1_cb(on_center_button);

    Front_Intake.set_brake_mode(MOTOR_BRAKE_COAST);
	Front_Intake.set_gearing(MOTOR_GEARSET_36);
	Back_Intake.set_brake_mode(MOTOR_BRAKE_COAST);
	Back_Intake.set_gearing(MOTOR_GEARSET_36);
	Middle_Intake.set_brake_mode(MOTOR_BRAKE_COAST);
	Middle_Intake.set_gearing(MOTOR_GEARSET_36);
	Back_Right.set_gearing(MOTOR_GEARSET_18);
	Back_Left.set_gearing(MOTOR_GEARSET_18);
	Front_optical.set_integration_time(10);
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
    double filtered_value = 20;
	bool colorState = false;
	int toggle_counter = 0;
	while (true) {
		std::string fmt_string = std::format("{}", Front_optical.get_proximity());
		master.set_text(1,1, fmt_string);
		//printf("Hue: %lf\n", Front_optical.get_hue());		

		//Front Intake Controlls
		if (master.get_digital(DIGITAL_R1)){
			Front_Intake.move_velocity(100);
			Back_Intake.move_velocity(100);
			Middle_Intake.move_velocity(100);
			Back_Right.move_voltage(13000);
			Back_Left.move_voltage(13000);
			//Ball Hue Values
			//red = 15
			//blue = 212
			// printf("Distance: %lf\t", Back_optical.get_proximity());
			// printf("Hue: %lf\n", Back_optical.get_hue());
			if (Front_optical.get_led_pwm() > 25 && colorState != (Front_optical.get_hue() > 100 && Front_optical.get_hue() < 200) && toggle_counter > 20){
			Front_Intake.set_brake_mode(MOTOR_BRAKE_HOLD);
			Back_Intake.set_brake_mode(MOTOR_BRAKE_HOLD);
				Middle_Intake.set_brake_mode(MOTOR_BRAKE_HOLD);
				Front_Intake.brake();
				Back_Intake.brake();
				Middle_Intake.brake();
				colorSort.set_value(!colorState);
				colorState = !colorState;
				toggle_counter = 0;
				// Middle_Intake.move_voltage(3000);
				pros::delay(40);
				// pros::delay(200);
				// Front_Intake.move_velocity(100);
				// Back_Intake.move_velocity(100);
				// Middle_Intake.move_velocity(100);
				// Back_Right.move_voltage(13000);
				// Back_Left.move_voltage(13000);
				Middle_Intake.set_brake_mode(MOTOR_BRAKE_COAST);
				Front_Intake.set_brake_mode(MOTOR_BRAKE_COAST);
				Back_Intake.set_brake_mode(MOTOR_BRAKE_COAST);
			}
		}
		else if (master.get_digital(DIGITAL_R2)){
			Front_Intake.move_velocity(100);
			Back_Intake.move_velocity(-100);
			Middle_Intake.move_velocity(-100);
		}
		else {
			Front_Intake.brake();
			Back_Intake.brake();
			Middle_Intake.brake();
			Back_Right.brake();
			Back_Left.brake();
			Back_optical.set_led_pwm(0);
		}

		if (master.get_digital(DIGITAL_L1)){
			Back_Intake.move_velocity(20);
			Front_Intake.move_velocity(100);
			Middle_Intake.move_velocity(-100);
		}

		else {
			
			Back_Intake.set_brake_mode(MOTOR_BRAKE_COAST);
		}

		if (Front_optical.get_proximity() > 240){
				Front_optical.set_led_pwm(50);
		}
		else{
			Front_optical.set_led_pwm(5);
		}
        
		//Pneumatic Controls
		if(master.get_digital_new_press(DIGITAL_L2)){
			extendoIntake.toggle();
		}
		else if (master.get_digital_new_press(DIGITAL_X)){
			rearIntake.toggle();
		}
		else if (master.get_digital_new_press(DIGITAL_A)){
			colorSort.toggle();
		}
		else if (master.get_digital_new_press(DIGITAL_B)){
			aligner.toggle();
		}

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		int strafe = master.get_analog(ANALOG_LEFT_X); // Gets the turn left/right from left joystick

        chassis.holonomic(dir, strafe, turn);
		
		//
		pros::delay(10);
		toggle_counter++;                               // Run for 20 ms then update
	}
}