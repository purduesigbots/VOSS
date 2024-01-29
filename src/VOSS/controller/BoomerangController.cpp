#include "VOSS/controller/BoomerangController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>

namespace voss::controller {

BoomerangController::BoomerangController(std::shared_ptr<localizer::AbstractLocalizer> l)
    : AbstractController(l) {
    child = std::make_shared<PIDController>(l);
}

chassis::ChassisCommand BoomerangController::get_command(bool reverse,
                                                         bool thru) {
	// TODO: finish
    if (target.theta > 360) {
        child->set_target(target, false);
    } else {
        Point current_pos = this->l->get_position();
        double dx = target.x - current_pos.x;
        double dy = target.y - current_pos.y;
        double distance_error = sqrt(dx * dx + dy * dy);
        double at = voss::to_radians(target.theta);
        Pose carrotPoint = {
            target.x - distance_error * cos(at) * lead_pct,
            target.y - distance_error * sin(at) * lead_pct,
            target.theta
        };
        //printf("current: %f %f\n", current_pos.x, current_pos.y);
        //printf("carrotPoint: %f %f\n", carrotPoint.x, carrotPoint.y);
        child->set_target(carrotPoint, false);
    }
    return child->get_command(reverse, thru);
}

chassis::ChassisCommand BoomerangController::get_angular_command(bool reverse, bool thru) {
    child->set_target(target, false);
    child->set_angular_target(angular_target, false);
    return child->get_angular_command(reverse, thru);
}

void BoomerangController::reset() {
    child->reset();
}

} // namespace voss::controller