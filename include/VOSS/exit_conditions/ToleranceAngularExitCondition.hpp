#pragma once

#include "VOSS/exit_conditions/AbstractExitCondition.hpp"

namespace voss::controller {

class ToleranceAngularExitCondition : public AbstractExitCondition {

  private:
    double tolerance;
    double tolerance_time;
    double current_time;

  public:
    ToleranceAngularExitCondition(double tolerance, double tolerance_time);
    bool is_met(Pose current_pose, bool thru) override;
    void reset() override;
};
} // namespace voss::controller