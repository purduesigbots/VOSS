#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>
#include <memory>

namespace voss::controller {

AbstractController::AbstractController(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
    this->l = l;
}

// Set desired postion with x, y, and heading
// Relative target position WIP
void AbstractController::set_target(Pose target, bool relative,
                                    std::shared_ptr<AbstractExitCondition> ec) {
    if (relative) {
        Point p = l->get_position();         // robot position
        double h = l->get_orientation_rad(); // robot heading in radians
        double x_new = p.x + target.x * cos(h) - target.y * sin(h);
        double y_new = p.y + target.x * sin(h) + target.y * cos(h);
        this->target = Pose{x_new, y_new, 361};
    } else {
        this->target = target;
    }

    ec->set_target(this->target);
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