#pragma once

#include <memory>

#include "SSOV/trajectory/Path.hpp"
#include "SSOV/trajectory/MotionProfile.hpp"
#include "SSOV/trajectory/Trajectory.hpp"

namespace ssov {

class PathTrajectory : public Trajectory{
    private:
        MotionProfile profile;
        std::shared_ptr<Path> path;
    public:
        PathTrajectory(std::shared_ptr<Path> path, const TrajectoryConstraints& constraints);
        TrajectoryState at(double t) override;
        double duration() override {
            return profile.duration();
        }
};

}