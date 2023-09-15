#pragma once

#include "AbstractChassis.hpp"
#include "ChassisCommand.hpp"

namespace legs::chassis {

class DiffChassis : public AbstractChassis {

public:
	DiffChassis(std::initializer_list<int8_t> left_motors,
	            std::initializer_list<int8_t> right_motors,
	            controller::AbstractController& default_controller);

	void tank(double left_speed, double right_speed);
	void arcade(double forward_speed, double turn_speed);

	bool execute(ChassisCommand cmd);
};

} // namespace legs::chassis