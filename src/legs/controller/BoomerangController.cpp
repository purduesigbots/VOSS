#include "legs/controller/BoomerangController.hpp"
#include "legs/chassis/ChassisCommand.hpp"
#include <cmath>

namespace legs::controller {

BoomerangController::BoomerangController(localizer::AbstractLocalizer& l)
    : PIDController(l) {
}

chassis::ChassisCommand BoomerangController::get_command(bool reverse,
                                                         bool thru) {
	// TODO: finish
	return chassis::ChassisCommand{chassis::Stop{}};
}

} // namespace legs::controller