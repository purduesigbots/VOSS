#pragma once

#include "AbstractController.hpp"
#include "PIDController.hpp"
#include "legs/chassis/ChassisCommand.hpp"
#include "legs/localizer/AbstractLocalizer.hpp"

namespace legs::controller {

class BoomerangController : public PIDController {
private:
	double lead_pct;

public:
	BoomerangController(localizer::AbstractLocalizer& l);

	chassis::ChassisCommand get_command(bool reverse, bool thru);

	friend class BoomerangControllerBuilder;
};

} // namespace legs::controller