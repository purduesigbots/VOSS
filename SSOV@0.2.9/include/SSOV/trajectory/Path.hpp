#pragma once

#include <vector>
#include <memory>

#include "SSOV/common/Pose.hpp"

namespace ssov {

struct PoseWithCurvature {
    Pose pose;
    double curvature;
};

struct PathSample {
    std::vector<PoseWithCurvature> poses;
    std::vector<double> distances;
};

class Path {
    private:
        bool reversed;
    public:
        Path(bool reversed): reversed(reversed) {};
        bool is_reversed() const {
            return reversed;
        }
        virtual double length() const = 0;
        virtual PoseWithCurvature at(double distance) const = 0;
        PathSample sample(double dist_resolution) const;
};

}