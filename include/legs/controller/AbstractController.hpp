#pragma once

#include "legs/chassis/ChassisCommand.hpp"
#include "legs/localizer/AbstractLocalizer.hpp"

namespace legs::controller {

class AbstractController {

protected:
	localizer::AbstractLocalizer* l;

public:
	AbstractController(localizer::AbstractLocalizer& l);

	virtual chassis::ChassisCommand get_command(Pose target);
	virtual chassis::ChassisCommand get_command(Point target);

	virtual void reset() = 0;
};

} // namespace legs::controller