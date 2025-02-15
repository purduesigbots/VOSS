#pragma once

#include "SSOV/exit_condition/ExitCondition.hpp"
#include "pros/rtos.hpp"
#include <cstdint>

namespace ssov {

class TimeoutExitCondition: public ExitCondition {
    private:
        const uint32_t timeout;
        uint32_t start_time;
    public:
        TimeoutExitCondition(uint32_t timeout): timeout(timeout) {};
        bool is_met(const Pose &current_pose, const Pose &target_pose, bool thru) override {
            return pros::millis() - start_time >= timeout;
        }
        void reset() override {
            start_time = pros::millis();
        }
};

}