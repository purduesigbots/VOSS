#pragma once

#include "voss/controller/PIDController.hpp"

namespace voss::controller {

class PIDControllerBuilder {
private:
	controller::PIDController ctrl;

public:
	PIDControllerBuilder(localizer::AbstractLocalizer& l);

	static PIDControllerBuilder newBuilder(localizer::AbstractLocalizer& l);

	PIDControllerBuilder& withLinearConstants(double kP, double kI, double kD);
	PIDControllerBuilder& withAngularConstants(double kP, double kI, double kD);
	PIDControllerBuilder& withTrackingKP(double kP);
	PIDControllerBuilder& withExitError(double error);
	PIDControllerBuilder& withMinError(double error);
	PIDControllerBuilder& withSettleTime(double time);

	PIDController build();
};

} // namespace voss::controller