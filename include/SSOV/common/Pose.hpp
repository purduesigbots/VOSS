#pragma once

#include <cmath>

namespace ssov {
    struct Point {
        double x;
        double y;
        Point(double x, double y): x(x), y(y) {};
        Point(): Point(0, 0) {};
    };

    struct Pose {
        double x;
        double y;
        double theta;
        Pose(double x, double y, double theta): x(x), y(y), theta(theta) {};
        Pose(): Pose(0, 0, 0) {};
        Point to_point() const {
            return {x, y};
        }
    };

    inline double distance(const Point a, const Point b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return sqrt(dx * dx + dy * dy);
    }
}