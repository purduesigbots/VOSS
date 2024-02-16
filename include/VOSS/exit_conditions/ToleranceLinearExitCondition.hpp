#pragma once

#include "VOSS/exit_conditions/AbstractExitCondition.hpp"

namespace voss::controller {

class ToleranceLinearExitCondition : public AbstractExitCondition {

  private:
    double tolerance;

  public:
    ToleranceLinearExitCondition(Pose target_pose, double tolerance);
    bool is_met(Pose current_pose);
};

} // namespace voss::controller