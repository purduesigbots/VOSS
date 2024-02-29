#pragma once

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include <atomic>
#include <memory>

namespace voss::localizer {

class ADILocalizer : public AbstractLocalizer {

  private:
    std::atomic<double> prev_left_pos, prev_right_pos, prev_middle_pos;
    AtomicPose prev_pose;

    std::atomic<double> left_right_tpi, middle_tpi;
    std::atomic<double> track_width;
    std::atomic<double> left_right_dist, middle_dist;
    std::atomic<int> imu_ports;

    std::unique_ptr<pros::adi::Encoder> left_encoder;
    std::unique_ptr<pros::adi::Encoder> right_encoder;
    std::unique_ptr<pros::adi::Encoder> middle_encoder;
    std::unique_ptr<pros::IMU> imu;

  public:
    ADILocalizer(int left, int right, int mid, double lr_tpi, double mid_tpi,
                 double track_width, double middle_dist, int imu_port);

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