#pragma once

#include "AbstractExitCondition.hpp"
#include "VOSS/utils/Pose.hpp"
#include <memory>
#include <vector>
#include <functional>

namespace voss::controller {

class ExitConditions : public AbstractExitCondition {

  private:
    std::vector<std::shared_ptr<controller::AbstractExitCondition>> conditions;

    ExitConditions();
  public:
    static ExitConditions new_conditions();

    void set_target(voss::Pose new_target) override;
    ExitConditions& add_settle(int settle_time, double tolerance, int initial_delay);
    ExitConditions& add_timeout(int timeout);
    ExitConditions& add_angular_tolerance(double tolerance);
    ExitConditions& add_linear_tolerance(double tolerance);
    ExitConditions& add_condition(std::shared_ptr<AbstractExitCondition> ec);

    std::shared_ptr<ExitConditions> exit_if(std::function<bool()> callback);

    bool is_met(voss::Pose current_pose);
    bool all_met(voss::Pose current_pose);

    std::shared_ptr<ExitConditions> build();

    void reset() override;
};

} // namespace voss::controller