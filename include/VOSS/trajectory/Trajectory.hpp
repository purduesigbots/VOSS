#pragma once

#include "VOSS/trajectory/SplinePath.hpp"
#include "VOSS/trajectory/Profile.hpp"
#include "VOSS/utils/Pose.hpp"

namespace voss::trajectory {

struct TrajectoryConstraints {
    double max_vel;
    double max_accel;
    double max_decel;
    double track_width;
};

struct TrajectoryPose {
    PoseWithCurvature pose;
    double vel;
    double acc;
};

class Trajectory {
  private:
    Profile profile;
    SplinePath path;
  public:
    Trajectory(SplinePath path, TrajectoryConstraints constraints);

    TrajectoryPose at(double t);
};

}