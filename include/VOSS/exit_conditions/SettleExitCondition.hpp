#pragma once

#include "VOSS/exit_conditions/AbstractExitCondition.hpp"

namespace voss::controller {

class SettleExitCondition : public AbstractExitCondition {

  private:
    int settle_time;
    int current_time;
    Pose prev_pose;
    double tolerance;

  public:
    SettleExitCondition(Pose target_pose, int settle_time, double tolerance);
    bool is_met(Pose current_pose);
};
} // namespace voss::controller