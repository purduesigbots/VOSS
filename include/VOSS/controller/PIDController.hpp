#pragma once

#include "voss/controller/AbstractController.hpp"

namespace voss::controller {

class PIDController : public AbstractController {
protected:
	double linear_kP, linear_kI, linear_kD;
	double angular_kP, angular_kI, angular_kD;
	double tracking_kP;
	double exit_error;

	double close;

	double prev_lin_err, total_lin_err, prev_ang_err, total_ang_err;

public:
	PIDController(localizer::AbstractLocalizer& l);

	double linear_pid(double error);
	double angular_pid(double error);

	chassis::ChassisCommand get_command(bool reverse, bool thru)override;
    chassis::ChassisCommand get_angular_command(bool reverse, bool thru)override;

	void reset();

	friend class PIDControllerBuilder;
};

} // namespace voss::controller