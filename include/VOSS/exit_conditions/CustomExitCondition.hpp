/**
 * @file CustomExitCondition.hpp
 * @brief
 * @version 0.1.2
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "../utils/Pose.hpp"
#include "AbstractExitCondition.hpp"
#include <functional>
namespace voss::controller {
class CustomExitCondition : public AbstractExitCondition {
  private:
    std::function<bool()> callback;

  public:
    CustomExitCondition(std::function<bool()> callback);
    bool is_met(Pose current_pose, bool thru) override;
};
} // namespace voss::controller
