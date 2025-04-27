#pragma once

#include "pros/imu.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/localizer/AbstractTrackingWheel.hpp"
#include "VOSS/utils/ekfilter.hpp"
#include <atomic>
#include <memory>

inline constexpr double meter_to_inch = 39.3701;
inline constexpr double inch_to_meter = 1.0 / meter_to_inch;

namespace voss::localizer {

class TrackingWheelLocalizer : public AbstractLocalizer {
  protected:
    std::atomic<double> prev_left_pos, prev_right_pos, prev_middle_pos, prev_theta;

    std::atomic<double> left_right_dist, middle_dist;




  public:
    TrackingWheelLocalizer(std::unique_ptr<AbstractTrackingWheel> left,
                           std::unique_ptr<AbstractTrackingWheel> right,
                           std::unique_ptr<AbstractTrackingWheel> middle,
                           std::vector<std::shared_ptr<pros::IMU>> imu,
                           double left_right_dist, double middle_dist);
    void update() override;
    void calibrate() override;
    void set_pose(Pose pose) override;
    void set_pose(double x, double y, double theta) override;
    void enable_gps(bool enable) {
        this->gps_enabled = enable;
    }



    friend class TrackingWheelLocalizerBuilder;
  public:
    std::shared_ptr<voss::EKFilter> kalman_filter;
    std::shared_ptr<pros::GPS> gps;
    std::unique_ptr<AbstractTrackingWheel> left_tracking_wheel,
        right_tracking_wheel, middle_tracking_wheel;
    std::vector<std::shared_ptr<pros::IMU>> imus;
    std::atomic_bool gps_enabled = true;
};

} // namespace voss::localizer