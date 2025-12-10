#pragma once

#include <cmath>
#include <algorithm>

namespace ssov {

// constexpr specifies it is possible to evaluate at compile time
// so angle unit conversions can be done at compile time
inline constexpr double to_radians(double degrees) {
    return degrees * M_PI / 180;
}

inline constexpr double to_degrees(double radians) {
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

inline double slew(double target, double current, double slew) {
    if (!slew) return target;
    // experience from 2025 skills: also slewing on deceleration
    // makes movement less jerky when coming to a stop
    /*
    if (target > current + slew && current >= 0) {
        target = current + slew;
    }
    else if (target < current - slew && current <= 0) {
        target = current - slew;
    }
    */
    target = std::clamp(target, current - slew, current + slew);
    return target;
}

}