#pragma once

#include <memory>
#include "SSOV/exit_condition/ExitCondition.hpp"
#include "SSOV/exit_condition/SettleExitCondition.hpp"
#include "SSOV/exit_condition/ToleranceExitCondition.hpp"
#include "SSOV/exit_condition/ThruToleranceExitCondition.hpp"

namespace ssov {
// simple exit condition to combine settle, tolerance, and thru_tolerance
class CombinedExitCondition: public ExitCondition {
    private:
        std::shared_ptr<SettleExitCondition> settle;
        std::shared_ptr<ToleranceExitCondition> tolerance;
        std::shared_ptr<ThruToleranceExitCondition> thru_tolerance;
    public:
        CombinedExitCondition(std::shared_ptr<SettleExitCondition> settle,
            std::shared_ptr<ToleranceExitCondition> tolerance,
            std::shared_ptr<ThruToleranceExitCondition> thru_tolerance):
            settle(settle),
            tolerance(tolerance),
            thru_tolerance(thru_tolerance) {};
        bool is_met(const Pose &current_pose, const Pose &target_pose, bool thru) override {
            bool settle_met = settle->is_met(current_pose, target_pose, thru);
            bool tolerance_met = tolerance->is_met(current_pose, target_pose, thru);
            bool thru_tolerance_met = thru_tolerance->is_met(current_pose, target_pose, thru);
            if (settle_met || tolerance_met || thru_tolerance_met) {
                printf("settle: %d tolerance: %d thru: %d\n", settle_met, tolerance_met, thru_tolerance_met);
            }
            return settle_met || tolerance_met || thru_tolerance_met;
        }
        void reset() override {
            settle->reset();
            tolerance->reset();
        }
};

}