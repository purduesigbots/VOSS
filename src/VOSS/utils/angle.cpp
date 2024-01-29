#include "VOSS/utils/angle.hpp"

#include <cmath>

namespace voss {

double to_radians(double degrees) {
    return degrees * M_PI / 180;
}

double to_degrees(double radians) {
    return radians * 180 * M_1_PI;
}

double norm(double radians) {
    radians = fmod(radians, 2 * M_PI);
    if (radians < 0)
        radians += 2 * M_PI;
    return radians;
}

double norm_delta(double radians) {
    return std::remainder(radians, 2 * M_PI);
}
} // namespace voss