#pragma once

#include "VOSS/utils/Pose.hpp"

namespace voss::controller {

class AbstractExitCondition {

  protected:
    Pose target_pose;

  public:
    virtual bool is_met(Pose current_pose) = 0;
    void set_target(Pose target);
};

} // namespace voss::controller