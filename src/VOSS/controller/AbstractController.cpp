#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>
#include <memory>

namespace voss::controller {

// Set desired postion with x, y, and heading
// Relative target position WIP
void AbstractController::set_target(const Pose& target) {
    this->target = target;
}

// Set desired orientation
void AbstractController::set_angular_target(double angular_target) {
    this->angular_target = angular_target;
}

} // namespace voss::controller