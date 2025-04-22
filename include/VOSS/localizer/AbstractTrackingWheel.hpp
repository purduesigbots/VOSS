#pragma once

#include <atomic>

namespace voss::localizer {

class AbstractTrackingWheel {
  protected:
    AbstractTrackingWheel();
    std::atomic<double> tpi;

  public:
    virtual double get_raw_position() = 0;
    double get_dist_travelled();
    double get_dist_travelled(double delta_ticks);
    double get_tpi() {
        return tpi;
    }
    void set_tpi(double new_tpi);
    virtual void reset() = 0;
};

} // namespace voss::localizer
