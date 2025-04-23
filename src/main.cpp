#include "main.h"
#include "pros/motors.h"
#include "VOSS/api.hpp"
#include "VOSS/localizer/IMELocalizerBuilder.hpp"

#define LEFT_DRIVE_MOTOR_PORTS {-1, 3, -4, -5, 6}
#define RIGHT_DRIVE_MOTOR_PORTS {20, -19, 18, -17, 16}

#define ADI_EXPANDER_PORT (13)
#define MIDDLE_TRACKER_PORT ('C') // and 'D'
#define LEFT_TRACKER_PORT ('E')   // and 'F'
#define RIGHT_TRACKER_PORT ('A')  // and 'B'

std::shared_ptr<voss::localizer::TrackingWheelLocalizer> odom =
    voss::localizer::TrackingWheelLocalizerBuilder::new_builder()
        .with_middle_encoder(ADI_EXPANDER_PORT, MIDDLE_TRACKER_PORT)
        .with_left_encoder(ADI_EXPANDER_PORT, LEFT_TRACKER_PORT)
        .with_right_encoder(ADI_EXPANDER_PORT, RIGHT_TRACKER_PORT)
        .with_middle_tpi(424.958333333)
        .with_left_tpi(420.25)
        .with_right_tpi(419.416666667)
        .with_track_width(4.21362825311)
        .with_middle_dist(-1.125)
        .build();

/*std::shared_ptr<voss::localizer::IMELocalizer> odom =
    voss::localizer::IMELocalizerBuilder::new_builder()
        .with_left_motors(LEFT_DRIVE_MOTOR_PORTS)
        .with_right_motors(RIGHT_DRIVE_MOTOR_PORTS)
        .with_left_right_tpi(200.0 / (3.0 * M_PI) * 37.2 / 24.0)
        .with_track_width(9.85)
        .build();*/

std::shared_ptr<voss::controller::PIDController> pid =
    voss::controller::PIDControllerBuilder::new_builder(odom)
        .with_linear_constants(12, 0, 90)
        .with_angular_constants(180, 0, 1500)
        .with_min_error(5)
        .build();

std::shared_ptr<voss::controller::ExitConditions> ec =
    voss::controller::ExitConditions::new_conditions()
        .add_settle(100, 0.25, 100)
        .build();

pros::Controller master(pros::E_CONTROLLER_MASTER);

voss::chassis::DiffChassis chassis(LEFT_DRIVE_MOTOR_PORTS,
                                   RIGHT_DRIVE_MOTOR_PORTS, pid, ec, 8,
                                   pros::E_MOTOR_BRAKE_COAST);

inline pros::adi::Encoder middle_encoder({ADI_EXPANDER_PORT,
                                          MIDDLE_TRACKER_PORT,
                                          MIDDLE_TRACKER_PORT + 1});
inline pros::adi::Encoder left_encoder({ADI_EXPANDER_PORT, LEFT_TRACKER_PORT,
                                        LEFT_TRACKER_PORT + 1});
inline pros::adi::Encoder right_encoder({ADI_EXPANDER_PORT, RIGHT_TRACKER_PORT,
                                         RIGHT_TRACKER_PORT + 1});

inline void print_odom() {
    auto p = odom->get_pose();
    /*std::cout << odom->left_tracking_wheel->get_raw_position() << ","
              << odom->right_tracking_wheel->get_raw_position() << std::endl;
    return;*/
    std::cout << "X: " << p.x << "\n"
              << "Y: " << p.y << "\n"
              << "A: " << voss::to_degrees(p.theta.value()) << "\n"
              << "H: " << voss::to_degrees(voss::norm(p.theta.value())) << "\n"
              << "M: " << odom->middle_tracking_wheel->get_dist_travelled()
              << " : " << odom->middle_tracking_wheel->get_raw_position()
              << "\n"
              << "L: " << odom->left_tracking_wheel->get_dist_travelled()
              << " : " << odom->left_tracking_wheel->get_raw_position() << "\n"
              << "R: " << odom->right_tracking_wheel->get_dist_travelled()
              << " : " << odom->right_tracking_wheel->get_raw_position() << "\n"
              << std::endl;
    return;
    std::cout << pros::millis() << ","
              << odom->middle_tracking_wheel->get_raw_position() << ","
              << odom->left_tracking_wheel->get_raw_position() << ","
              << odom->right_tracking_wheel->get_raw_position() << ","
              << p.theta.value() << std::endl;
}

void initialize() {
    odom->begin_localization();
    pros::Task([] {
        while (true) {
            if (false || master.get_digital_new_press(DIGITAL_UP)) {
                print_odom();
            }
            pros::delay(10);
        }
    });
}

inline bool auton_wait() {
    chassis.arcade(0, 0);
    while (!master.get_digital(DIGITAL_X) && !master.get_digital(DIGITAL_B)) {
        pros::delay(10);
    }
    return master.get_digital(DIGITAL_B);
}

void autonomous() {
    std::cout << "Starting\n";
    print_odom();
    chassis.move(24, 25);
    std::cout << "After first forward\n";
    print_odom();
    auton_wait();
    chassis.turn(-90, 25);
    std::cout << "After first turn\n";
    print_odom();
    auton_wait();

    chassis.move(24, 25);
    std::cout << "After second forward\n";
    print_odom();
    auton_wait();
    chassis.turn(-180, 25);
    std::cout << "After second turn\n";
    print_odom();
    auton_wait();

    chassis.move(24, 25);
    std::cout << "After third forward\n";
    print_odom();
    auton_wait();
    chassis.turn(-270, 25);
    std::cout << "After third turn\n";
    print_odom();
    auton_wait();

    chassis.move(24, 25);
    std::cout << "After fourth forward\n";
    print_odom();
    auton_wait();
    chassis.turn(0, 25);
    std::cout << "After fourth turn\n";
    print_odom();
}

void opcontrol() {
    chassis.set_brake_mode(MOTOR_BRAKE_COAST);
    while (true) {
        chassis.arcade(master.get_analog(ANALOG_LEFT_Y),
                       master.get_analog(ANALOG_RIGHT_X));

        if (master.get_digital_new_press(DIGITAL_X)) {
            autonomous();
        }

        pros::delay(10);
    }
}
