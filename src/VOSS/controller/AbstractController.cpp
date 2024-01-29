#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>

namespace voss::controller {

AbstractController::AbstractController(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
	this->l = l;
}

void AbstractController::set_target(Pose target, bool relative) {
	if (relative) {
		Point p = l->get_position();         // robot position
		double h = l->get_orientation_rad(); // robot heading in radians
		double x_new = p.x + target.x * cos(h) - target.y * sin(h);
		double y_new = p.y + target.x * sin(h) + target.y * cos(h);
		this->target = Pose{x_new, y_new, 361};
	} else {
		this->target = target;
	}
}

void AbstractController::set_angular_target(double angular_target,
                                            bool relative) {
	angular_target = voss::to_radians(angular_target);
	if (relative) {
		this->angular_target =
		    voss::norm(angular_target + this->l->get_orientation_rad());
	} else {
		this->angular_target = voss::norm(angular_target);
	}
}

} // namespace voss::controller