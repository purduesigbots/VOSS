#pragma once

#include "VOSS/trajectory/SplinePath.hpp"
#include "VOSS/trajectory/Profile.hpp"
#include "VOSS/utils/Pose.hpp"
#include "AbstractTrajectory.hpp"

namespace voss::trajectory {
class Trajectory: public AbstractTrajectory{
  private:
    Profile profile;
    SplinePath path;
  public:
    Trajectory(SplinePath path, TrajectoryConstraints constraints);

    TrajectoryPose at(double t) override;
    double get_duration() override;
};

}