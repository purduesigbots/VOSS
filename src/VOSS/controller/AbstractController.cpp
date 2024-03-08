#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>

namespace voss::controller {

AbstractController::AbstractController(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
    this->l = l;
}

// Set desired postion with x, y, and heading
// Relative target position WIP
void AbstractController::set_target(Pose target, bool relative) {
    if (relative) {
        Point p = l->get_position();         // robot position
        double h = l->get_orientation_deg(); // robot heading in radians
        double x_new = p.x + target.x * cos(h) - target.y * sin(h);
        double y_new = p.y + target.x * sin(h) + target.y * cos(h);
        if (target.theta.has_value()) {
            this->target = Pose{x_new, y_new, target.theta.value() + h};
        } else {
            this->target = Pose{x_new, y_new, std::nullopt};
        }
    } else {
        this->target = target;
    }
}

// Set desired orientation
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