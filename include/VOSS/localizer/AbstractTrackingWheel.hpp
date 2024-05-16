/**
 * @file AbstractTrackingWheel.hpp
 * @brief
 * @version 0.1.2
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <atomic>

namespace voss::localizer {

class AbstractTrackingWheel {
  protected:
    AbstractTrackingWheel();
    std::atomic<double> tpi;
    virtual double get_raw_position() = 0;

  public:
    double get_dist_travelled();
    void set_tpi(double new_tpi);
    virtual void reset() = 0;
};

} // namespace voss::localizer
