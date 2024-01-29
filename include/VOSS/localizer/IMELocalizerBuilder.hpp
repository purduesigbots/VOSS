#pragma once

#include "IMELocalizer.hpp"

namespace voss::localizer {

class IMELocalizerBuilder {

    std::vector<int8_t> left_motors, right_motors, horizontal_motors;
    double lr_tpi, mid_tpi;
    double track_width;
    double middle_dist;
    int imu_port;

  public:
    IMELocalizerBuilder();

    static IMELocalizerBuilder new_builder();

    IMELocalizerBuilder& with_left_motors(std::vector<int8_t> m);
    IMELocalizerBuilder& with_right_motors(std::vector<int8_t> m);
    IMELocalizerBuilder& with_horizontal_motors(std::vector<int8_t> m);
    IMELocalizerBuilder& with_middle_tpi(double mid_tpi);
    IMELocalizerBuilder& with_left_right_tpi(double lr_tpi);
    IMELocalizerBuilder& with_track_width(double track_width);
    IMELocalizerBuilder& with_middle_distance(double middle_dist);
    IMELocalizerBuilder& with_imu(int imu_port);

    std::shared_ptr<IMELocalizer> build();
};

} // namespace voss::localizer