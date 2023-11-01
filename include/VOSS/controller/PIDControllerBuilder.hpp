#pragma once

#include "voss/controller/PIDController.hpp"

namespace voss::controller {

class PIDControllerBuilder {
private:
	controller::PIDController ctrl;

public:
	PIDControllerBuilder(std::shared_ptr<localizer::AbstractLocalizer> l);

	static PIDControllerBuilder
	newBuilder(std::shared_ptr<localizer::AbstractLocalizer> l);

	PIDControllerBuilder& withLinearConstants(double kP, double kI, double kD);
	PIDControllerBuilder& withAngularConstants(double kP, double kI, double kD);
	PIDControllerBuilder& withTrackingKP(double kP);
	PIDControllerBuilder& withExitError(double error);
	PIDControllerBuilder& withAngularExitError(double error);
	PIDControllerBuilder& withMinError(double error);

	PIDController build();
};

} // namespace voss::controller