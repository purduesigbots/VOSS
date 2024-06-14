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

    bool inline operator==(const Point& b) const {
        return this->x == b.x && this->y == b.y;
    }

    Point inline operator+(const Point& b) const {
        return {this->x + b.x, this->y + b.y};
    }

    Point inline operator-(const Point& b) const {
        return {this->x - b.x, this->y - b.y};
    }

    double inline operator*(const Point& b) const {
        return this->x * b.x + this->y * b.y;
    }
};

} // namespace voss