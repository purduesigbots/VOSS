#include "voss/controller/PIDControllerBuilder.hpp"
#include "VOSS/utils/angle.hpp"
#include "voss/controller/PIDController.hpp"
#include "voss/localizer/AbstractLocalizer.hpp"

namespace voss::controller {

PIDControllerBuilder::PIDControllerBuilder(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : ctrl(l) {
}

PIDControllerBuilder PIDControllerBuilder::newBuilder(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
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

PIDControllerBuilder& PIDControllerBuilder::withAngularExitError(double error) {
	this->ctrl.angular_exit_error = voss::to_radians(error);
	return *this;
}

PIDControllerBuilder& PIDControllerBuilder::withMinError(double error) {
	this->ctrl.min_error = error;
	return *this;
}

PIDControllerBuilder& PIDControllerBuilder::withSettleTime(double time) {
	this->ctrl.settle_time = time;
	return *this;
}

PIDController PIDControllerBuilder::build() {
	return this->ctrl;
}

} // namespace voss::controller