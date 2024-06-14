
#include "../include/VOSS/trajectory/PreGenTrajectory.hpp"

namespace voss::trajectory {
PreGenTrajectory::PreGenTrajectory(
    const std::map<double, TrajectoryPose>& t_to_trajectory)
    : t_to_trajectory(t_to_trajectory) {
}

TrajectoryPose PreGenTrajectory::at(double t) {
    return t_to_trajectory.at(t);
}

double PreGenTrajectory::get_duration() {
    return t_to_trajectory.crbegin()->first;
}
}; // namespace voss::trajectory
