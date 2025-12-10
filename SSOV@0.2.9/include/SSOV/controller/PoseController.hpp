#pragma once

#include "SSOV/chassis/ChassisCommand.hpp"
#include "SSOV/common/Pose.hpp"

namespace ssov {
class PoseController {
    public:
        virtual DriveSignal compute(const Pose &current_pose, const Pose &target_pose, bool reverse, bool thru, bool holonomic, float strafe_angle=0) = 0;
        virtual void reset() = 0;
};
}