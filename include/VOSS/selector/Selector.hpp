#pragma once

#include "liblvgl/lvgl.h"

namespace voss::selector {

void init(int hue, int default_auton, const char** autons);
int get_auton();

void destroy();

} // namespace voss::selector