#pragma once

#include "SSOV/chassis/DiffChassis.hpp"
#include "SSOV/trajectory/Trajectory.hpp"

namespace ssov {

class TrajectoryFollower {
    public:
        virtual ChassisSpeeds compute(Pose current_pose, Pose current_velocities, TrajectoryState target_state) = 0;
};

}