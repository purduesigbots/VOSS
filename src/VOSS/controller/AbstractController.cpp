#include "voss/controller/AbstractController.hpp"
#include <cmath>

namespace voss::controller {

AbstractController::AbstractController(localizer::AbstractLocalizer& l) {
	this->l = &l;
}

void AbstractController::set_target(Pose target, bool relative) {
	if (relative) {
		Point p = l->get_position();         // robot position
		double h = l->get_orientation_rad(); // robot heading in radians
		double x_new = p.x + target.x * cos(h) - target.y * sin(h);
		double y_new = p.y + target.x * sin(h) + target.y * cos(h);
		this->target = Pose{x_new, y_new, 0};
	} else {
		this->target = target;
	}
}

} // namespace voss::controller