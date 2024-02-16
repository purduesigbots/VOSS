#pragma once

#include "AbstractExitCondition.hpp"
#include "VOSS/utils/Pose.hpp"
#include <memory>
#include <vector>

namespace voss::controller {

class ExitConditions {

  private:
    std::vector<std::shared_ptr<controller::AbstractExitCondition>> conditions;
    voss::Pose target_pose;

    ExitConditions();

  public:
    static ExitConditions new_conditions();

    ExitConditions& set_target(voss::Pose new_target);
    ExitConditions& add_settle(int settle_time, double tolerance);
    ExitConditions& add_timeout(int timeout);
    ExitConditions& add_angular_tolerance(double tolerance);
    ExitConditions& add_linear_tolerance(double tolerance);
    ExitConditions& add_condition(AbstractExitCondition& ec);

    bool is_met(voss::Pose current_pose);
    bool all_met(voss::Pose current_pose);
};

} // namespace voss::controller