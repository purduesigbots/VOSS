#pragma once

namespace voss::selector {

void init(int default_auton, const char** autons);
int get_auton();

} // namespace voss::selector