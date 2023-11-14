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
BoomerangControllerBuilder::withLinearConstants(double kP, double kI,
                                                double kD) {
	this->ctrl.child->linear_kP = kP;
	this->ctrl.child->linear_kI = kI;
	this->ctrl.child->linear_kD = kD;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withAngularConstants(double kP, double kI,
                                                 double kD) {
	this->ctrl.child->angular_kP = kP;
	this->ctrl.child->angular_kI = kI;
	this->ctrl.child->angular_kD = kD;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withTrackingKP(double kP) {
	this->ctrl.child->tracking_kP = kP;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withExitError(double error) {
	this->ctrl.child->exit_error = error;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withAngularExitError(double error) {
	this->ctrl.child->angular_exit_error = error;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withMinError(double error) {
	this->ctrl.child->min_error = error;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withLeadPct(double lead_pct) {
	this->ctrl.lead_pct = lead_pct;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withSettleTime(double time) {
	this->ctrl.child->settle_time = (time > 0) ? time : 250;
	return *this;
}

BoomerangController BoomerangControllerBuilder::build() {
	return this->ctrl;
}

} // namespace voss::controller