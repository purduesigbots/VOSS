#include "VOSS/controller/BoomerangControllerBuilder.hpp"

#include <utility>
#include "VOSS/controller/BoomerangController.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/utils/angle.hpp"

namespace voss::controller {

BoomerangControllerBuilder::BoomerangControllerBuilder(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : ctrl(std::move(l)) {
    this->ctrl.p = nullptr;
}

BoomerangControllerBuilder BoomerangControllerBuilder::new_builder(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
    BoomerangControllerBuilder builder(std::move(l));
    return builder;
}

BoomerangControllerBuilder
BoomerangControllerBuilder::from(BoomerangController bmr) {
    BoomerangControllerBuilder builder(bmr.l);
    builder.ctrl = bmr;
    return builder;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_linear_constants(double kP, double kI,
                                                  double kD) {
    this->ctrl.linear_kP = kP;
    this->ctrl.linear_kI = kI;
    this->ctrl.linear_kD = kD;
    return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_angular_constants(double kP, double kI,
                                                   double kD) {
    this->ctrl.angular_kP = kP;
    this->ctrl.angular_kI = kI;
    this->ctrl.angular_kD = kD;
    return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_exit_error(double error) {
    this->ctrl.exit_error = error;
    return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_angular_exit_error(double error) {
    this->ctrl.angular_exit_error = voss::to_radians(error);
    return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_min_error(double error) {
    this->ctrl.min_error = error;
    return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_lead_pct(double lead_pct) {
    this->ctrl.lead_pct = lead_pct;
    return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_settle_time(double time) {
    this->ctrl.settle_time = (time > 0) ? time : 250;
    return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_min_vel_for_thru(double min_vel) {
    this->ctrl.min_vel = min_vel;
    return *this;
}

std::shared_ptr<BoomerangController> BoomerangControllerBuilder::build() {
    return std::make_shared<BoomerangController>(this->ctrl);
}

} // namespace voss::controller