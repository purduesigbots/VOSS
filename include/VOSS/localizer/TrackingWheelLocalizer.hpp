#pragma once

#include "pros/imu.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/localizer/AbstractTrackingWheel.hpp"
#include <atomic>
#include <memory>

namespace voss::localizer {

class TrackingWheelLocalizer : public AbstractLocalizer {
  protected:
    std::atomic<double> prev_left, prev_right, prev_middle;

    std::atomic<double> left_distance, right_distance, middle_dist;
    std::vector<std::unique_ptr<pros::IMU>> imu;
    Pose offset = {0, 0, 0.0};
    std::atomic<double> horizontal_offset;

  public:
    TrackingWheelLocalizer(std::unique_ptr<AbstractTrackingWheel> left,
                           std::unique_ptr<AbstractTrackingWheel> right,
                           std::unique_ptr<AbstractTrackingWheel> middle,
                           std::vector<std::unique_ptr<pros::IMU>> imu,
                           double left_distance, double right_distance, double middle_dist,
                           Pose offset);
    void update() override;
    void calibrate() override;
    void set_pose(Pose pose) override;
    void set_pose(double x, double y, double theta) override;
    void set_horizontal_offset(double horizontal_offset);

    std::unique_ptr<AbstractTrackingWheel> left_tracking_wheel,
        right_tracking_wheel, middle_tracking_wheel;
    friend class TrackingWheelLocalizerBuilder;
};

} // namespace voss::localizer
