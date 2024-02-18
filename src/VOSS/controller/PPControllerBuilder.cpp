#include "VOSS/controller/PPControllerBuilder.hpp"
#include "PPController.hpp"
#include "VOSS/utils/angle.hpp"
#include <memory>

namespace voss::controller {
PPControllerBuilder::PPControllerBuilder(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : ctrl(l) {
}

PPControllerBuilder PPControllerBuilder::new_builder(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
    PPControllerBuilder builder(l);
    return builder;
}

PPControllerBuilder&
PPControllerBuilder::with_linear_constants(double kP, double kI, double kD) {
    this->ctrl.child->linear_kP = kP;
    this->ctrl.child->linear_kI = kI;
    this->ctrl.child->linear_kD = kD;
    return *this;
}

PPControllerBuilder&
PPControllerBuilder::with_angular_constants(double kP, double kI, double kD) {
    this->ctrl.child->angular_kP = kP;
    this->ctrl.child->angular_kI = kI;
    this->ctrl.child->angular_kD = kD;
    return *this;
}

PPControllerBuilder& PPControllerBuilder::with_tracking_kp(double kP) {
    this->ctrl.child->tracking_kP = kP;
    return *this;
}

PPControllerBuilder& PPControllerBuilder::with_exit_error(double error) {
    this->ctrl.child->exit_error = error;
    return *this;
}

PPControllerBuilder&
PPControllerBuilder::with_angular_exit_error(double error) {
    this->ctrl.child->angular_exit_error = voss::to_radians(error);
    return *this;
}

PPControllerBuilder& PPControllerBuilder::with_min_error(double error) {
    this->ctrl.child->min_error = error;
    return *this;
}

PPControllerBuilder&
PPControllerBuilder::with_lookahead_distance(double lookaheadDist) {
    this->ctrl.lookAheadDist = lookaheadDist;
    return *this;
}

PPControllerBuilder& PPControllerBuilder::with_settle_time(double time) {
    this->ctrl.child->settle_time = (time > 0) ? time : 250;
    return *this;
}

std::shared_ptr<PPController> PPControllerBuilder::build() {
    return std::make_shared<PPController>(this->ctrl);
}

} // namespace voss::controller