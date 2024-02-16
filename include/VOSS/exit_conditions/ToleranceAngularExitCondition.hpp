#pragma once

#include "VOSS/exit_conditions/AbstractExitCondition.hpp"

namespace voss::controller {

class ToleranceAngularExitCondition : public AbstractExitCondition {

  private:
    double tolerance;

  public:
    ToleranceAngularExitCondition(Pose target_pose, double tolerance);
    bool is_met(Pose current_pose);
};
} // namespace voss::controller