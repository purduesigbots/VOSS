
#pragma once
#include "AbstractTrajectory.hpp"
#include <map>
#include <vector>

namespace voss::trajectory {
class PreGenTrajectory : public AbstractTrajectory {
  public:
    PreGenTrajectory(const std::map<double, TrajectoryPose>& t_to_trajectory);
    TrajectoryPose at(double t) override;
    double get_duration() override;
  private:
    std::map<double, TrajectoryPose> t_to_trajectory;
};
}; // namespace voss::trajectory
