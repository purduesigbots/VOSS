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