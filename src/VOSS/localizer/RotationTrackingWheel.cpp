#include "VOSS/localizer/RotationTrackingWheel.hpp"

namespace voss::localizer {

RotationTrackingWheel::RotationTrackingWheel(int port) : AbstractTrackingWheel() {
    this->encoder = std::make_unique<pros::v5::Rotation>(port);
}

double RotationTrackingWheel::get_raw_position() {
    return this->encoder->get_position();
}

void RotationTrackingWheel::reset() {
    this->encoder->reset_position();
}

}