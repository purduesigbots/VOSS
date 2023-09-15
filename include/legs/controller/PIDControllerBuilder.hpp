#pragma once

#include "legs/controller/PIDController.hpp"

namespace legs::controller {

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
	PIDControllerBuilder& withLeadPct(double lead_pct);

	PIDController build();
};

} // namespace legs::controller