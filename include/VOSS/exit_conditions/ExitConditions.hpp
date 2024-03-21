#pragma once

#include "AbstractExitCondition.hpp"
#include "ToleranceExitCondition.hpp"
#include "VOSS/utils/Pose.hpp"
#include <functional>
#include <memory>
#include <vector>

namespace voss::controller {

class ExitConditions : public AbstractExitCondition {

  private:
    std::vector<std::shared_ptr<controller::AbstractExitCondition>> conditions;
    ExitConditions();

  public:
    static ExitConditions new_conditions();

    void set_target(voss::Pose new_target) override;
    ExitConditions& add_settle(int settle_time, double tolerance,
                               int initial_delay);
    ExitConditions& add_timeout(int timeout);
    ExitConditions& add_tolerance(double linear_tolerance,
                                  double angular_tolerance,
                                  double tolerance_time);
    ExitConditions& add_thru_smoothness(double smoothness);
    ExitConditions& add_custom_condition(std::function<bool()> callback);
    ExitConditions& add_condition(std::shared_ptr<AbstractExitCondition> ec);

    std::shared_ptr<ExitConditions> exit_if(std::function<bool()> callback);

    bool is_met(voss::Pose current_pose, bool thru);
    bool all_met(voss::Pose current_pose, bool thru);

    std::shared_ptr<ExitConditions> build();

    void reset() override;
};

} // namespace voss::controller