#pragma once

#include "AbstractController.hpp"
#include "PIDController.hpp"
#include "voss/chassis/ChassisCommand.hpp"
#include "voss/localizer/AbstractLocalizer.hpp"

namespace voss::controller {

class BoomerangController : public PIDController {
private:
	double lead_pct;

public:
	BoomerangController(localizer::AbstractLocalizer& l);

	chassis::ChassisCommand get_command(bool reverse, bool thru);

	friend class BoomerangControllerBuilder;
};

} // namespace voss::controller