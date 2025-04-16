#include "main.h"
#include "VOSS/api.hpp"
#include "VOSS/localizer/IMELocalizerBuilder.hpp"

#define LEFT_DRIVE_MOTOR_PORTS {-10, -9, 8, -7, 6}
#define RIGHT_DRIVE_MOTOR_PORTS {20, -19, 18, -17, 16}

#define ADI_EXPANDER_PORT (11)
#define MIDDLE_TRACKER_PORT ('A') // and 'B'
#define LEFT_TRACKER_PORT ('C')   // and 'D'
#define RIGHT_TRACKER_PORT ('E')  // and 'F'

std::shared_ptr<voss::localizer::TrackingWheelLocalizer> odom =
    voss::localizer::TrackingWheelLocalizerBuilder::new_builder()
        .with_middle_encoder(ADI_EXPANDER_PORT, MIDDLE_TRACKER_PORT)
        .with_left_encoder(ADI_EXPANDER_PORT, LEFT_TRACKER_PORT)
        .with_right_encoder(ADI_EXPANDER_PORT, RIGHT_TRACKER_PORT)
        // .with_middle_tpi(419.44790793)
        .with_middle_tpi(415.874775083)
        // .with_left_tpi(415.874775083)
        .with_left_tpi(416.0625)
        .with_right_tpi(415.604166667)
        // .with_right_tpi(408.479084953)
        .with_track_width(3.495)
        .with_middle_dist(-2.8)
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
        .with_min_vel_for_thru(50)
        .build();

std::shared_ptr<voss::controller::ExitConditions> ec =
    voss::controller::ExitConditions::new_conditions()
        .add_settle(100, 0.25, 100)
        .add_timeout(5000)
        .add_thru_smoothness(8)
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
}

void initialize() {
    odom->begin_localization();
    pros::Task([] {
        while (true) {
            if (master.get_digital_new_press(DIGITAL_UP)) {
                print_odom();
            }
            pros::delay(50);
        }
    });
}

void opcontrol() {
    while (true) {
        chassis.arcade(master.get_analog(ANALOG_LEFT_Y),
                       master.get_analog(ANALOG_RIGHT_X));

        pros::delay(10);
    }
}
