/**
 * @file TimeOutExitCondition.hpp
 * @brief
 * @version 0.1.2
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */
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
