#include <vector>

#include "SSOV/trajectory/Trajectory.hpp"

namespace ssov {

class CombinedTrajectory: public Trajectory {
    private:
        std::vector<Trajectory*> trajectories;
        Trajectory* current_trajectory;
        int current_trajectory_index = 0;
        double time_offset = 0;
        double d;
    public:
        CombinedTrajectory(std::initializer_list<Trajectory*> trajs): trajectories(trajs) {
            current_trajectory = trajectories.front();
            d = 0;
            for (const auto &traj : trajs) {
                d += traj->duration();
            }
        }
        TrajectoryState at(double t) override {
            double offset_time = t - time_offset;
            if (offset_time < current_trajectory->duration()) {
                return current_trajectory->at(offset_time);
            } else {
                time_offset += current_trajectory->duration();
                current_trajectory = trajectories[++current_trajectory_index];
                offset_time = t - time_offset;
                return current_trajectory->at(offset_time);
            }
        }
        double duration() override {
            return d;
        }
};

}