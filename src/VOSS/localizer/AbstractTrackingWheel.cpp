#include "VOSS/localizer/AbstractTrackingWheel.hpp"

namespace voss::localizer {

AbstractTrackingWheel::AbstractTrackingWheel() {
}

double AbstractTrackingWheel::get_dist_travelled() {
    return this->get_raw_position() / this->tpi;
}

void AbstractTrackingWheel::set_tpi(double new_tpi) {
    this->tpi = new_tpi;
}

} // namespace voss::localizer
