#pragma once

#include "AbstractController.hpp"
#include "PIDController.hpp"
#include "voss/chassis/ChassisCommand.hpp"
#include "voss/localizer/AbstractLocalizer.hpp"

namespace voss::controller {

class BoomerangController : public AbstractController {
protected:
	std::shared_ptr<PIDController> child = nullptr;
	double lead_pct;

public:
	BoomerangController(localizer::AbstractLocalizer& l);

	chassis::ChassisCommand get_command(bool reverse, bool thru)override;
	chassis::ChassisCommand get_angular_command(bool reverse, bool thru)override;

	void reset();

	friend class BoomerangControllerBuilder;
};

} // namespace voss::controller