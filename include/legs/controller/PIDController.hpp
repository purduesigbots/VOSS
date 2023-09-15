#pragma once

#include "legs/controller/AbstractController.hpp"

namespace legs::controller {

class PIDController : public AbstractController {
private:
	double linear_kP, linear_kI, linear_kD;
	double angular_kP, angular_kI, angular_kD;
	double tracking_kP;
	double exit_error;
	double lead_pct;

	double prev_lin_err, total_lin_err, prev_ang_err, total_ang_err;

public:
	PIDController(localizer::AbstractLocalizer& l);

	double linear_pid(double error);
	double angular_pid(double error);

	chassis::ChassisCommand get_command(Point target);
	void reset();

	friend class PIDControllerBuilder;
};

} // namespace legs::controller