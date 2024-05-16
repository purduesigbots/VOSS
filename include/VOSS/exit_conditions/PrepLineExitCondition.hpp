/**
 * @file PrepLineExitCondition.hpp
 * @brief
 * @version 0.1.2
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "AbstractExitCondition.hpp"
#include "VOSS/utils/Pose.hpp"

namespace voss::controller {
class PrepLineExitCondition : public AbstractExitCondition {
  private:
    double thru_smoothness;

  public:
    PrepLineExitCondition(double thru_smoothness);
    bool is_met(voss::Pose pose, bool thru) override;
};
} // namespace voss::controller
