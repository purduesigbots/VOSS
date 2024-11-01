#pragma once

#include <cstdint>

#include "SSOV/exit_condition/ExitCondition.hpp"

namespace ssov {
class ToleranceExitCondition : public ExitCondition {
    private:
        const double linear_tolerance;
        const double angular_tolerance;
        const uint32_t tolerance_time;
        uint32_t in_tolerance_time;
        uint32_t prev_time;
    public:
        ToleranceExitCondition(const double linear_tolerance,
                               const double angular_tolerance,
                               const uint32_t tolerance_time):
            linear_tolerance(linear_tolerance),
            angular_tolerance(angular_tolerance),
            tolerance_time(tolerance_time) {};
        bool is_met(const Pose &current_pose, const Pose &target_pose, const bool &thru) override;
        void reset() override;
};
}