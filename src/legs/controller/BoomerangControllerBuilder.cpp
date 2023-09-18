#include "legs/controller/BoomerangControllerBuilder.hpp"
#include "legs/controller/BoomerangController.hpp"
#include "legs/localizer/AbstractLocalizer.hpp"

namespace legs::controller {

BoomerangControllerBuilder::BoomerangControllerBuilder(
    localizer::AbstractLocalizer& l)
    : ctrl(l) {
}

BoomerangControllerBuilder
BoomerangControllerBuilder::newBuilder(localizer::AbstractLocalizer& l) {
	BoomerangControllerBuilder builder(l);
	return builder;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withLinearConstants(double kP, double kI,
                                                double kD) {
	this->ctrl.linear_kP = kP;
	this->ctrl.linear_kI = kI;
	this->ctrl.linear_kD = kD;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withAngularConstants(double kP, double kI,
                                                 double kD) {
	this->ctrl.angular_kP = kP;
	this->ctrl.angular_kI = kI;
	this->ctrl.angular_kD = kD;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withTrackingKP(double kP) {
	this->ctrl.tracking_kP = kP;
	return *this;
}

BoomerangControllerBuilder&
BoomerangControllerBuilder::withExitError(double error) {
	this->ctrl.exit_error = error;
	return *this;
}

BoomerangController BoomerangControllerBuilder::build() {
	return this->ctrl;
}

} // namespace legs::controller