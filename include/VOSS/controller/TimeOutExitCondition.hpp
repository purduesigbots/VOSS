#pragma once

#include "VOSS/controller/AbstractExitCondition.hpp"

namespace voss::controller {
    
    class TimeOutExitCondition : public AbstractExitCondition{

  private:
  Pose target_pose;
  int timeout;
  int current_time;

  public:
  bool is_met(Pose current_pose);
  void set_target(Pose target);
  void set_timeout(int timeout);
    
};
}