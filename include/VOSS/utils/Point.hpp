/**
 * @file Point.hpp
 * @brief
 * @version 0.1.2
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <cmath>

namespace voss {

struct Point {
    double x;
    double y;

    static double getDistance(Point a, Point b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return sqrt(dx * dx + dy * dy);
    }
};

} // namespace voss
