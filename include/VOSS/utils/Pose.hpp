#pragma once

#include "Point.hpp"
#include <atomic>
#include <cmath>
#include <optional>

namespace voss {

struct AtomicPose;
struct Pose {
    double x;
    double y;
    std::optional<double> theta = std::nullopt;

    static Pose get_relative(const Pose& a, const Pose& current_pose) {
        double h = current_pose.theta.value();
        double x_new = current_pose.x + a.x * cos(h) - current_pose.y * sin(h);
        double y_new = current_pose.y + a.x * sin(h) + current_pose.y * cos(h);
        if (a.theta.has_value()) {
            return Pose{x_new, y_new, a.theta.value() + h};
        }
        return Pose{x_new, y_new, std::nullopt};
    }

    void operator=(const Point& pt) {
        x = pt.x;
        y = pt.y;
        theta = std::nullopt;
    }
};

struct AtomicPose {
    std::atomic<double> x;
    std::atomic<double> y;
    std::atomic<double> theta;

    void operator=(const Pose& pose) {
        x = pose.x;
        y = pose.y;
        if (pose.theta.has_value()) {
            theta = pose.theta.value();
        } else {
            theta = NAN;
        }
    }

    void operator=(const AtomicPose& pose) {
        x = pose.x.load();
        y = pose.y.load();
        theta = pose.theta.load();
    }
};

} // namespace voss