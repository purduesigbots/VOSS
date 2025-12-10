#pragma once

#include "SSOV/exit_condition/ToleranceExitCondition.hpp"

#include <cstdint>

namespace ssov {

class SettleExitCondition: public ExitCondition {
    private:
        const double linear_tolerance;
        const double angular_tolerance;
        const uint32_t settle_time;
        const uint32_t initial_delay;
        uint32_t initial_time;
        uint32_t settled_time;
        uint32_t prev_time;
        Pose prev_pose;
    public:
        SettleExitCondition(const double linear_tolerance,
                            const double angular_tolerance,
                            const uint32_t initial_delay,
                            const uint32_t settle_time):
            linear_tolerance(linear_tolerance),
            angular_tolerance(angular_tolerance),
            initial_delay(initial_delay),
            settle_time(settle_time) {};
        bool is_met(const Pose &current_pose, const Pose &target_pose, bool thru) override;
        void reset() override;
};

}