#pragma once

#include "VOSS/utils/Pose.hpp"

namespace voss::controller {

class AbstractExitCondition {

  protected:
    Pose target_pose;
    bool target_has_coordinate() const;
    bool target_has_heading() const;

  public:
    virtual bool is_met(Pose current_pose, bool thru) = 0;
    virtual void set_target(Pose target);
    virtual void reset();
};

} // namespace voss::controller