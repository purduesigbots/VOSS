#pragma once

#include "Eigen/Dense"
#include <atomic>
#include <cmath>
#include <optional>

namespace voss {

struct AtomicPose;
struct Pose {
    double x;
    double y;
    std::optional<double> theta = std::nullopt;

    void operator=(const Eigen::Vector3d& other) {
        x = other.x();
        y = other.y();
        theta = other.z();
    }

    operator Eigen::Vector3d() const {
        return {x, y, theta.value_or(0.0)};
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

    void operator=(const Eigen::Vector3d& other) {
        x = other.x();
        y = other.y();
        theta = other.z();
    }

    operator Eigen::Vector3d() const {
        return {x.load(), y.load(), theta.load()};
    }
};

} // namespace voss