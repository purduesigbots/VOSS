#include "voss/controller/BoomerangControllerBuilder.hpp"
#include "voss/controller/BoomerangController.hpp"
#include "voss/localizer/AbstractLocalizer.hpp"

namespace voss::controller {

BoomerangControllerBuilder::BoomerangControllerBuilder(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : ctrl(l) {
}

BoomerangControllerBuilder BoomerangControllerBuilder::new_builder(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
	BoomerangControllerBuilder builder(l);
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
BoomerangControllerBuilder::with_tracking_kp(double kP) {
	this->ctrl.tracking_kP = kP;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_exit_error(double error) {
	this->ctrl.exit_error = error;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_lead_pct(double lead_pct) {
	this->ctrl.lead_pct = lead_pct;
	return *this;
}

BoomerangController BoomerangControllerBuilder::build() {
	return this->ctrl;
}

} // namespace voss::controller