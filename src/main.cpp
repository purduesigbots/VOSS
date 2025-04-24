#include "main.h"
#include "pros/motors.h"
#include "pros/screen.hpp"
#include "VOSS/api.hpp"

#define LEFT_DRIVE_MOTOR_PORTS {-1, 3, -4, -5, 6}
#define RIGHT_DRIVE_MOTOR_PORTS {20, -19, 18, -17, 16}

#define ADI_EXPANDER_PORT (22)
#define MIDDLE_TRACKER_PORT ('E')
#define LEFT_TRACKER_PORT ('C')
#define RIGHT_TRACKER_PORT ('A')

#define MIDDLE_TRACKER_PORT_2 ('G')
#define LEFT_TRACKER_PORT_2 ('A')

std::shared_ptr<voss::localizer::TrackingWheelLocalizer> odom =
    voss::localizer::TrackingWheelLocalizerBuilder::new_builder()
        .with_middle_encoder(ADI_EXPANDER_PORT, MIDDLE_TRACKER_PORT)
        .with_left_encoder(ADI_EXPANDER_PORT, LEFT_TRACKER_PORT)
        .with_right_encoder(ADI_EXPANDER_PORT, RIGHT_TRACKER_PORT)
        .with_middle_tpi(5000 / 24.0)
        .with_left_tpi(4988 / 24.0)
        .with_right_tpi(4988 / 24.0)
        .with_track_width(4.0625)
        .with_middle_dist(-4.625)
        .build();

std::shared_ptr<voss::localizer::TrackingWheelLocalizer> odom2 =
    voss::localizer::TrackingWheelLocalizerBuilder::new_builder()
        .with_middle_encoder(ADI_EXPANDER_PORT, MIDDLE_TRACKER_PORT_2)
        .with_left_encoder(ADI_EXPANDER_PORT, LEFT_TRACKER_PORT_2)
        .with_middle_tpi(300.0 * 13.4 / 12.0 * 11.95 / 12.0 * 21.7 / 24.0 *
                         26.2 / 24.0)
        .with_left_tpi(300.0 * 13.4 / 12.0 * 11.95 / 12.0)
        .with_track_width(2.375 * 2.0)
        .with_middle_dist(4.625)
        .with_imus({7, 10})
        .build();

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
// inline pros::adi::Encoder right_encoder({ADI_EXPANDER_PORT,
// RIGHT_TRACKER_PORT, RIGHT_TRACKER_PORT + 1});

inline void print_odom() {
    auto p = odom->get_pose();
    auto p2 = odom2->get_pose();
    // std::cout << p.x << "," << p.y << "," << p.theta.value() << "," << p2.x
    // << "," << p2.y << "," << p2.theta.value() << std::endl;
    // return;
    std::cout << "X1: " << p.x << "\n"
              << "Y1: " << p.y << "\n"
              << "H1: " << voss::to_degrees(p.theta.value()) << "\n"
              << "M1: " << odom->middle_tracking_wheel->get_raw_position()
              << "\n"
              << "L1: " << odom->left_tracking_wheel->get_raw_position() << "\n"
              << "R1: " << odom->right_tracking_wheel->get_raw_position()
              << "\n"
              << "X2: " << p2.x << "\n"
              << "Y2: " << p2.y << "\n"
              << "H2: " << voss::to_degrees(p2.theta.value()) << "\n"
              << std::endl;
    return;
    /*std::cout << odom->left_tracking_wheel->get_raw_position() << ","
              << odom->right_tracking_wheel->get_raw_position() <<
    std::endl; return;*/
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
    pros::IMU imu1(7);
    pros::IMU imu2(10);
    imu1.reset();
    imu2.reset();
    pros::delay(5000);
    std::cout << "imu1: " << imu1.get_heading()
              << " imu2: " << imu2.get_heading() << "\n";
    // return;

    odom->begin_localization();
    odom2->begin_localization();
    pros::Task([] {
        while (true) {
            if (true ||
                pros::screen::touch_status().touch_status ==
                    pros::E_TOUCH_HELD ||
                master.get_digital_new_press(DIGITAL_UP)) {
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
