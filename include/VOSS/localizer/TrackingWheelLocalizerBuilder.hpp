#pragma once

#include "VOSS/localizer/TrackingWheelLocalizer.hpp"

namespace voss::localizer {

class TrackingWheelLocalizerBuilder {
  private:
    double left_right_dist;
    double middle_dist;
    std::unique_ptr<AbstractTrackingWheel> left_tracking_wheel;
    std::unique_ptr<AbstractTrackingWheel> right_tracking_wheel;
    std::unique_ptr<AbstractTrackingWheel> middle_tracking_wheel;
    std::unique_ptr<pros::IMU> imu;

  public:
    TrackingWheelLocalizerBuilder();

    static TrackingWheelLocalizerBuilder new_builder();

    TrackingWheelLocalizerBuilder& with_left_encoder(int adi_port);
    TrackingWheelLocalizerBuilder& with_left_encoder(int smart_port,
                                                     int adi_port);
    TrackingWheelLocalizerBuilder& with_left_rotation(int port);
    TrackingWheelLocalizerBuilder& with_left_motor(int port);
    TrackingWheelLocalizerBuilder& with_right_encoder(int adi_port);
    TrackingWheelLocalizerBuilder& with_right_encoder(int smart_port,
                                                      int adi_port);
    TrackingWheelLocalizerBuilder& with_right_rotation(int port);
    TrackingWheelLocalizerBuilder& with_right_motor(int port);
    TrackingWheelLocalizerBuilder& with_middle_encoder(int adi_port);
    TrackingWheelLocalizerBuilder& with_middle_encoder(int smart_port,
                                                       int adi_port);
    TrackingWheelLocalizerBuilder& with_middle_rotation(int port);
    TrackingWheelLocalizerBuilder& with_middle_motor(int port);
    TrackingWheelLocalizerBuilder& with_left_right_tpi(double tpi);
    TrackingWheelLocalizerBuilder& with_middle_tpi(double tpi);
    TrackingWheelLocalizerBuilder& with_track_width(double track_width);
    TrackingWheelLocalizerBuilder& with_middle_dist(double middle_dist);
    TrackingWheelLocalizerBuilder& with_imu(int port);

    std::shared_ptr<TrackingWheelLocalizer> build();
};

} // namespace voss::localizer