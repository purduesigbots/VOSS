#pragma once

#include "VOSS/exit_conditions/AbstractExitCondition.hpp"

namespace voss::controller {

class TimeOutExitCondition : public AbstractExitCondition {

  private:
    int timeout;
    int current_time;

  public:
    TimeOutExitCondition(int timeout);
    bool is_met(Pose current_pose, bool thru) override;
    void reset() override;
};
} // namespace voss::controller