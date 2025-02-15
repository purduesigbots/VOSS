#pragma once

#include "SSOV/common/Pose.hpp"

namespace ssov {
class ExitCondition {
    public:
        virtual bool is_met(const Pose &current_pose, const Pose &target_pose, bool thru) = 0;
        virtual void reset() = 0;
};
}