#pragma once

#include "pros/gps.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include <atomic>
#include <memory>

namespace voss::localizer {

class GPSLocalizer : public AbstractLocalizer {

  private:
    std::atomic<int> gps_port;
    std::atomic<double> theta_offset = 0;
    std::atomic<double> x_offset = 0;
    std::atomic<double> y_offset = 0;
    std::unique_ptr<pros::GPS> gps;

  public:
    explicit GPSLocalizer(int gps_port);
    constexpr inline static double mm_to_inch(double cm);

    void update() override;
    void calibrate() override;

    void set_pose(Pose pose);
};

} // namespace voss::localizer