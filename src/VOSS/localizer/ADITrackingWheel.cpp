#include "VOSS/localizer/ADITrackingWheel.hpp"

namespace voss::localizer {

ADITrackingWheel::ADITrackingWheel(int adi_port) : AbstractTrackingWheel() {
    this->encoder = std::make_unique<pros::adi::Encoder>(abs(adi_port), abs(adi_port) + 1, adi_port < 0);
}

ADITrackingWheel::ADITrackingWheel(int smart_port, int adi_port) : AbstractTrackingWheel() {
    this->encoder = std::make_unique<pros::adi::Encoder>(std::make_tuple(smart_port, abs(adi_port), abs(adi_port) + 1), adi_port < 0);
}

double ADITrackingWheel::get_raw_position() {
    return this->encoder->get_value();
}

void ADITrackingWheel::reset() {
    this->encoder->reset();
}

}