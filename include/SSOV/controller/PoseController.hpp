#pragma once

#include "SSOV/chassis/ChassisCommand.hpp"
#include "SSOV/common/Pose.hpp"

namespace ssov {
class PoseController {
    public:
        virtual DriveSignal compute(Pose current_pose, Pose target_pose) = 0;
        virtual void reset() = 0;
};
}