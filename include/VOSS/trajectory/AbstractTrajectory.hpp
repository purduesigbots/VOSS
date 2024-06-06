
#pragma once

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

class AbstractTrajectory {
  public:
    virtual TrajectoryPose at(double t) = 0;
    virtual double get_duration() = 0;
};

} // namespace voss::trajectory
