#pragma once

#include "VOSS/controller/AbstractExitCondition.hpp"

namespace voss::controller {
    
    class SettleExitCondition : public AbstractExitCondition {

  private:
    Pose current_pose;
    int settle_time;
    int current_time;
    Pose prev_pose;
    double tolerance;

  public:
    bool is_met(Pose current_pose);
    void set_settle_time(int settle_time);
    void set_tolerance(double tolerance);
    
    };
}