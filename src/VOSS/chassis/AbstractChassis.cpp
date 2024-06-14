#include "VOSS/chassis/AbstractChassis.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "VOSS/asset/Decode.hpp"
#include "VOSS/constants.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/utils/angle.hpp"

#include <cmath>

namespace voss::chassis {

AbstractChassis::AbstractChassis(
    std::shared_ptr<voss::controller::PIDController> default_controller,
    std::shared_ptr<voss::localizer::AbstractLocalizer> l, ec_ptr ec) {
    this->l = std::move(l);
    this->default_controller = std::move(default_controller);
    this->default_ec = std::move(ec);
}

void AbstractChassis::move(double distance, double max, voss::Flags flags) {
    printf("im here0\n");
    this->move({distance, 0}, this->default_controller, this->default_ec, max,
               flags | voss::Flags::RELATIVE);
}

void AbstractChassis::move(Pose target, double max, voss::Flags flags) {
    printf("im here3\n");
    this->move(target, this->default_controller, this->default_ec, max, flags);
}

void AbstractChassis::turn(double target, double max, voss::Flags flags,
                           voss::AngularDirection direction) {
    this->turn(target, this->default_controller, this->default_ec, max, flags,
               direction);
}

void AbstractChassis::turn_to(Point target, double max, voss::Flags flags,
                              voss::AngularDirection direction) {
    this->turn_to(target, this->default_controller, this->default_ec, max,
                  flags, direction);
}


Pose AbstractChassis::process_target_pose(Pose target, bool relative) {
    if (target.theta.has_value()) {
        target.theta = voss::to_radians(target.theta.value());
    }
    if (relative) {
        return Pose::get_relative(target, this->l->get_pose());
    }
    return target;
}

double AbstractChassis::process_target_angle(double angle, bool relative) {
    angle = voss::to_radians(angle);
    if (relative) {
        return voss::norm(angle + this->l->get_orientation_rad());
    }
    return voss::norm(angle);
}

std::vector<Pose>
AbstractChassis::process_target_path(const std::vector<Pose>& path) {
    for (auto it : path) {
        if (it.theta.has_value()) {
            it.theta = voss::to_radians(it.theta.value());
        }
    }
    return path;
}

} // namespace voss::chassis