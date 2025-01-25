#pragma once

#include "SSOV/chassis/ChassisCommand.hpp"
#include "SSOV/common/Pose.hpp"

namespace ssov {
class PointController {
    public:
        virtual DriveSignal compute(const Pose &current_pose, const Point &target_point, bool reverse, bool thru) = 0;
        virtual void reset() = 0;
};
}