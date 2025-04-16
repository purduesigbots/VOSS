#pragma once

#include "ADILocalizer.hpp"
#include <memory>

namespace voss::localizer {

class ADILocalizerBuilder {

    int left, right, mid, imu_port;
    double lr_tpi, mid_tpi;
    double track_width;
    double middle_dist;

  public:
    ADILocalizerBuilder();

    static ADILocalizerBuilder new_builder();

    ADILocalizerBuilder& with_left_encoder(int c);
    ADILocalizerBuilder& with_right_encoder(int c);
    ADILocalizerBuilder& with_middle_encoder(int c);
    ADILocalizerBuilder& with_left_right_tpi(double lr_tpi);
    ADILocalizerBuilder& with_middle_tpi(double mid_tpi);
    ADILocalizerBuilder& with_track_width(double track_width);
    ADILocalizerBuilder& with_middle_distance(double middle_dist);
    ADILocalizerBuilder& with_imu(int imu_port);

    std::shared_ptr<ADILocalizer> build();
};

} // namespace voss::localizer