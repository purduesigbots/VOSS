#pragma once

#include "api.h"

namespace voss::selector {

class Selector {

private:
	int auton;
	int autonCount;
	const char* btnmMap[11] = {"", "", "", "", "", "",
	                           "", "", "", "", ""}; // up to 10 autons

	Selector();

public:
	Selector& get_instance() {
		static Selector instance;

		return instance;
	}

	int get_auton();
	void set_autons(const char** autons);
};

} // namespace voss::selector