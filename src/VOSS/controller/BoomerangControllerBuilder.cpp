#include "voss/controller/BoomerangControllerBuilder.hpp"
#include "voss/controller/BoomerangController.hpp"
#include "voss/localizer/AbstractLocalizer.hpp"
#include "VOSS/utils/angle.hpp"

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
	this->ctrl.child->linear_kP = kP;
	this->ctrl.child->linear_kI = kI;
	this->ctrl.child->linear_kD = kD;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_angular_constants(double kP, double kI,
                                                   double kD) {
	this->ctrl.child->angular_kP = kP;
	this->ctrl.child->angular_kI = kI;
	this->ctrl.child->angular_kD = kD;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_tracking_kp(double kP) {
	this->ctrl.child->tracking_kP = kP;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_exit_error(double error) {
	this->ctrl.child->exit_error = error;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_angular_exit_error(double error) {
	this->ctrl.child->angular_exit_error = voss::to_radians(error);
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_min_error(double error) {
	this->ctrl.child->min_error = error;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_lead_pct(double lead_pct) {
	this->ctrl.lead_pct = lead_pct;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::with_settle_time(double time) {
	this->ctrl.child->settle_time = (time > 0) ? time : 250;
	return *this;
}

BoomerangController BoomerangControllerBuilder::build() {
	return this->ctrl;
}

} // namespace voss::controller