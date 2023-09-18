#include "legs/controller/PIDControllerBuilder.hpp"
#include "legs/controller/PIDController.hpp"
#include "legs/localizer/AbstractLocalizer.hpp"

namespace legs::controller {

PIDControllerBuilder::PIDControllerBuilder(localizer::AbstractLocalizer& l)
    : ctrl(l) {
}

PIDControllerBuilder
PIDControllerBuilder::newBuilder(localizer::AbstractLocalizer& l) {
	PIDControllerBuilder builder(l);
	return builder;
}

PIDControllerBuilder&
PIDControllerBuilder::withLinearConstants(double kP, double kI, double kD) {
	this->ctrl.linear_kP = kP;
	this->ctrl.linear_kI = kI;
	this->ctrl.linear_kD = kD;
	return *this;
}

PIDControllerBuilder&
PIDControllerBuilder::withAngularConstants(double kP, double kI, double kD) {
	this->ctrl.angular_kP = kP;
	this->ctrl.angular_kI = kI;
	this->ctrl.angular_kD = kD;
	return *this;
}

PIDControllerBuilder& PIDControllerBuilder::withTrackingKP(double kP) {
	this->ctrl.tracking_kP = kP;
	return *this;
}

PIDControllerBuilder& PIDControllerBuilder::withExitError(double error) {
	this->ctrl.exit_error = error;
	return *this;
}

PIDController PIDControllerBuilder::build() {
	return this->ctrl;
}

} // namespace legs::controller