#include "SSOV/localizer/AbstractTrackingWheel.hpp"

namespace ssov {

AbstractTrackingWheel::AbstractTrackingWheel() {
}

double AbstractTrackingWheel::get_dist_travelled() {
    return this->get_raw_position() / this->tpi;
}

void AbstractTrackingWheel::set_tpi(double new_tpi) {
    this->tpi = new_tpi;
}

}