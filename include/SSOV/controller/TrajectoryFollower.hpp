#pragma once

#include "SSOV/chassis/ChassisCommand.hpp"
#include "SSOV/trajectory/Trajectory.hpp"

namespace ssov {

class TrajectoryFollower {
    public:
        virtual DriveSignal compute(Pose current_pose, Pose current_velocities, TrajectoryState target_state) = 0;
};

}