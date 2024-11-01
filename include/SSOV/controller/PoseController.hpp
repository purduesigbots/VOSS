#pragma once

#include "SSOV/chassis/DiffChassis.hpp"
#include "SSOV/common/Pose.hpp"

namespace ssov {
class PoseController {
    public:
        virtual ChassisSpeeds compute(Pose current_pose, Pose target_pose) = 0;
        virtual void reset() = 0;
};
}