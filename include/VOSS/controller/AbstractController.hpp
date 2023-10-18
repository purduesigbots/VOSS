#pragma once

#include "voss/chassis/ChassisCommand.hpp"
#include "voss/localizer/AbstractLocalizer.hpp"

namespace voss::controller {

class AbstractController {

protected:
	localizer::AbstractLocalizer* l;
	Pose target;
    double angular_target;

public:
	AbstractController(localizer::AbstractLocalizer& l);

	virtual chassis::ChassisCommand get_command(bool reverse, bool thru) = 0;
    virtual chassis::ChassisCommand get_angular_command(bool reverse, bool thru) = 0;

	virtual void reset() = 0;

	void set_target(Pose target, bool relative);
    void set_angular_target(double angle, bool relative);
};

} // namespace voss::controller