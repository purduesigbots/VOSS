#pragma once

#include "VOSS/utils/Pose.hpp"

namespace voss::controller {

class AbstractExitCondition {

  protected:
    Pose target_pose;

  public:
    virtual bool is_met(Pose current_pose) = 0;
    virtual void set_target(Pose target);
    virtual void reset();
};

} // namespace voss::controller