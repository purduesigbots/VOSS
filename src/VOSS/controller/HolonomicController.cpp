#include "VOSS/controller/HolonomicController.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>
#include <memory>

namespace voss::controller {

HolonomicController::HolonomicController(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
    this->l = l;
}

// Set desired postion with x, y, and heading
// Relative target position WIP
void HolonomicController::set_target(Pose target, bool relative,
                                    std::shared_ptr<AbstractExitCondition> ec) {
    if (target.theta.has_value()) {
        target.theta = voss::to_radians(*target.theta);
    }
    if (relative) {
        Point p = l->get_position();         // robot position
        double h = l->get_orientation_rad(); // robot heading in radians
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

    ec->set_target(this->target);
}

// Set desired orientation
void HolonomicController::set_angular_target(double angular_target,
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