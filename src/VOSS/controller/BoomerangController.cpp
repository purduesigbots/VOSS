#include "voss/controller/BoomerangController.hpp"
#include "voss/chassis/ChassisCommand.hpp"
#include <cmath>

namespace voss::controller {

BoomerangController::BoomerangController(localizer::AbstractLocalizer& l)
    : PIDController(l) {
}

chassis::ChassisCommand BoomerangController::get_command(bool reverse,
                                                         bool thru) {
	// TODO: finish
	return PIDController::get_command(reverse, thru);
}

} // namespace voss::controller