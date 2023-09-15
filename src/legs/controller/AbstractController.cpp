#include "legs/controller/AbstractController.hpp"

namespace legs::controller {

AbstractController::AbstractController(localizer::AbstractLocalizer& l) {
	this->l = &l;
}

chassis::ChassisCommand AbstractController::get_command(Pose target) {
	return chassis::ChassisCommand{chassis::Stop{}};
}

chassis::ChassisCommand AbstractController::get_command(Point target) {
	return chassis::ChassisCommand{chassis::Stop{}};
}

} // namespace legs::controller