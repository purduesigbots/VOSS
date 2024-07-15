#pragma once
#include "ExitConditions.hpp"

namespace voss::controller {
class ExitConditionsBuilder {
  public:
    static ExitConditionsBuilder new_builder();

    ExitConditionsBuilder& add_settle(int settle_time, double tolerance,
                               int initial_delay);
    ExitConditionsBuilder& add_timeout(int timeout);
    ExitConditionsBuilder& add_tolerance(double linear_tolerance,
                                  double angular_tolerance,
                                  double tolerance_time);
    ExitConditionsBuilder& add_thru_smoothness(double smoothness);
    ExitConditionsBuilder& add_custom_condition(std::function<bool()> callback);
    ExitConditionsBuilder& add_condition(std::shared_ptr<AbstractExitCondition> ec);

    std::shared_ptr<ExitConditions> build();



  private:
    ExitConditionsBuilder() = default;
    std::shared_ptr<ExitConditions> ec_ptr{};
};
}; // namespace voss::controller
