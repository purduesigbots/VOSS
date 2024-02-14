#pragma once

#include "VOSS/utils/flags.hpp"
#include <cmath>

namespace voss {

inline double to_radians(double degrees) {
    return degrees * M_PI / 180;
}

inline double to_degrees(double radians) {
    return radians * 180 * M_1_PI;
}

inline double norm(double radians) {
    radians = fmod(radians, 2 * M_PI);
    if (radians < 0)
        radians += 2 * M_PI;
    return radians;
}

inline double norm_delta(double radians) {
    return std::remainder(radians, 2 * M_PI);
}

} // namespace voss