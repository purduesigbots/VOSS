#pragma once

#include "SSOV/chassis/DiffChassis.hpp"
#include "SSOV/common/Pose.hpp"

namespace ssov {
class PointController {
    public:
        virtual ChassisSpeeds compute(const Pose &current_pose, const Point &target_point, const bool &reverse, const bool &thru) = 0;
        virtual void reset() = 0;
};
}