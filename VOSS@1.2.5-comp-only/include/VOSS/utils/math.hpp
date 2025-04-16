#pragma once
#include "pros/imu.hpp"
#include <cmath>
#include <numeric>
#include <vector>

namespace voss {

double get_avg(const std::vector<double>& vec) {
    if(vec.empty()) {
        return 0.0;
    }
    return std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
}
}; // namespace voss
