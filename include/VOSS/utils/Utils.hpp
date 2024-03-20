#pragma once
#include "pros/rtos.h"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include <functional>

namespace voss::utils {
void wait_until(std::function<bool()> callback) {
    while (!callback()) {
        pros::delay(10);
    }
}

}; // namespace voss::utils