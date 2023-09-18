#pragma once

#include "legs/chassis/ChassisCommand.hpp"
#include "legs/localizer/AbstractLocalizer.hpp"

namespace legs::controller {

class AbstractController {

protected:
	localizer::AbstractLocalizer* l;
	Pose target;

public:
	AbstractController(localizer::AbstractLocalizer& l);

	virtual chassis::ChassisCommand get_command(bool reverse, bool thru) = 0;

	virtual void reset() = 0;

	void set_target(Pose target, bool relative);
};

} // namespace legs::controller