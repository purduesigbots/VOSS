#pragma once

#include <cmath>
#include "SSOV/common/Math.hpp"

namespace ssov {
    struct Point {
        double x;
        double y;

        Point(double x, double y): x(x), y(y) {};
        Point(): Point(0, 0) {};

        Point operator+(const Point &other) const {
            return {x + other.x, y + other.y};
        }
        Point operator-(const Point &other) const {
            return {x - other.x, y - other.y};
        }
        Point rotate(double radians) const {
            double c = cos(radians);
            double s = sin(radians);
            return {x * c - y * s, x * s + y * c};
        }
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

        Pose rotate(double radians) const {
            double c = cos(radians);
            double s = sin(radians);
            return {x * c - y * s, x * s + y * c, theta + radians};
        }
        Pose operator+(const Pose &other) const {
            Pose rotated = other.rotate(theta);
            return {x + rotated.x, y + rotated.y, rotated.theta};
        }
        Pose operator-(const Pose &other) const {
            double angle = theta - other.theta;
            Point point = to_point() - other.to_point().rotate(angle);
            return {point.x, point.y, angle};
        }
        Pose operator+(const Point &other) const {
            Point rotated = other.rotate(theta);
            return {x + rotated.x, y + rotated.y, theta};
        }
        Pose operator-(const Point &other) const {
            Point rotated = other.rotate(theta);
            return {x - rotated.x, y - rotated.y, theta};
        }
    };

    struct UserPose {
        double x;
        double y;
        double theta_deg;
        UserPose(double x, double y, double theta_deg): x(x), y(y), theta_deg(theta_deg) {};
        UserPose(): UserPose(0, 0, 0) {};
        Pose to_pose() const {
            return {x, y, to_radians(theta_deg)};
        }
    };

    inline double distance(const Point a, const Point b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return sqrt(dx * dx + dy * dy);
    }
}