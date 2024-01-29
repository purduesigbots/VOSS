#pragma once

#include <atomic>

namespace voss {

struct Pose {
    double x;
    double y;
    double theta;
};

struct AtomicPose {
    std::atomic<double> x;
    std::atomic<double> y;
    std::atomic<double> theta;

    void operator=(const Pose& pose) {
        x = pose.x;
        y = pose.y;
        theta = pose.theta;
    }
};

} // namespace voss