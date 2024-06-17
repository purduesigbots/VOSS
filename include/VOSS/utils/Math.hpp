#pragma once
#include "cmath"
#include <numeric>

namespace voss {

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename It> double average(It begin, It end) {
    auto dist = std::distance(begin, end);
    if (dist == 0)
        return 0.0;
    auto s = std::accumulate(begin, end, 0.0);
    return s / dist;
}

double linear_vel_to_rpm(double velocity_inches_per_sec, double wheel_diameter) {
    return (velocity_inches_per_sec / (M_PI * wheel_diameter)) * 60;
}

}; // namespace voss