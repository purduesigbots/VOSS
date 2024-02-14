#pragma once

#include "VOSS/controller/AbstractExitCondition.hpp"

namespace voss::controller {

    class ToleranceLinearExitCondition : public AbstractExitCondition {

    private:
        Pose current_pose;
        Pose target_pose;
        double tolerance;

    public:
        bool is_met(Pose current_pose);
        void set_target(Pose target);
        void set_tolerance(double tolerance);
    };
} // namespace voss::controller