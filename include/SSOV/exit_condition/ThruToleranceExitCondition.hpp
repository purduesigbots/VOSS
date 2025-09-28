#pragma once

#include "SSOV/exit_condition/ExitCondition.hpp"

namespace ssov {
// simple exit condition for thru movements, returns true when thru is true
// and within linear tolerance of the target pose.
class ThruToleranceExitCondition: public ExitCondition {
    private:
        const double linear_tolerance;
    public:
        ThruToleranceExitCondition(double linear_tolerance): linear_tolerance(linear_tolerance) {};
        bool is_met(const Pose &current_pose, const Pose &target_pose, bool thru) override {
            if (thru) {
                double lin_error = distance(target_pose.to_point(), current_pose.to_point());
                return lin_error < linear_tolerance;
            }
            return false;
        }
        void reset() override {}
};

}