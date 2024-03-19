#pragma once

#include <atomic>
#include <cmath>
#include <optional>

namespace voss {

struct AtomicPose;
struct Pose {
    double x;
    double y;
    std::optional<double> theta = std::nullopt;
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