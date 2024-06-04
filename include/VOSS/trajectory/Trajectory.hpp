#pragma once

#include "VOSS/trajectory/SplinePath.hpp"
#include "VOSS/trajectory/Profile.hpp"
#include "VOSS/utils/Pose.hpp"

namespace voss::trajectory {

struct TrajectoryConstraints {
    double max_vel;
    double max_accel;
    double max_decel;
    double max_ang_accel;
    double max_centr_accel;
    double track_width;
};

struct TrajectoryPose {
    Pose pose;
    double vel;
    double acc;
    double ang_vel;
};

class Trajectory {
  private:
    Profile profile;
    SplinePath path;
  public:
    Trajectory(SplinePath path, TrajectoryConstraints constraints);

    TrajectoryPose at(double t);
    double duration();
};

}