#pragma once

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include <memory>

namespace voss::localizer {

class IMELocalizer : public AbstractLocalizer {

  private:
    double prev_left_pos, prev_right_pos, prev_middle_pos;
    AtomicPose prev_pose;

    double left_right_tpi, middle_tpi;
    double track_width;
    double left_right_dist, middle_dist;
    int imu_ports;

    std::unique_ptr<pros::MotorGroup> left_motors;
    std::unique_ptr<pros::MotorGroup> right_motors;
    std::unique_ptr<pros::MotorGroup> horizontal_motors;
    std::unique_ptr<pros::IMU> imu;

  public:
    IMELocalizer(std::vector<int8_t> left_motors_ports,
                 std::vector<int8_t> right_motors_ports,
                 std::vector<int8_t> horizontal_motors_ports, double lr_tpi,
                 double mid_tpi, double track_width, double middle_dist,
                 int imu_port);

    double get_left_encoder_value();
    double get_right_encoder_value();
    double get_middle_encoder_value();
    double get_imu_value();

    void update() override;
    void calibrate() override;

    void set_pose(Pose pose) override;
    void set_pose(double x, double y, double theta) override;
};

} // namespace voss::localizer