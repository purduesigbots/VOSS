#pragma once

#include <initializer_list>
#include <vector>
#include "VOSS/trajectory/SplineSegment.hpp"
#include "VOSS/utils/Integration.hpp"
#include "VOSS/utils/Pose.hpp"

namespace voss::trajectory {

struct PoseWithCurvature {
    Pose pose;
    double curvature;
};

struct PathSample {
    std::vector<PoseWithCurvature> poses;
    std::vector<double> distances;
};

class SplinePath {
  private:
    std::vector<SplineSegment> x_segments;
    std::vector<SplineSegment> y_segments;
    voss::utils::IntegralScan arc_length_reparam;

  public:
    SplinePath(std::initializer_list<Pose> waypoints);
    SplinePath(std::vector<Pose> waypoints);

    double length();
    PoseWithCurvature at(double distance);

    PathSample sample(double dist_resolution);
};

}