#include "VOSS/exit_conditions/ExitConditionsBuilder.hpp"
#include "CustomExitCondition.hpp"
#include "PrepLineExitCondition.hpp"
#include "SettleExitCondition.hpp"
#include "TimeOutExitCondition.hpp"

namespace voss::controller {
ExitConditionsBuilder ExitConditionsBuilder::new_builder() {
    return {};
}

ExitConditionsBuilder& ExitConditionsBuilder::add_settle(int settle_time, double tolerance,
                                           int initial_delay) {
    SettleExitCondition ec(settle_time, tolerance, initial_delay);
    ec_ptr->conditions.push_back(std::make_shared<SettleExitCondition>(ec));
    return *this;
}

ExitConditionsBuilder& ExitConditionsBuilder::add_timeout(int timeout) {
    ec_ptr->conditions.push_back(std::make_shared<TimeOutExitCondition>(timeout));
    return *this;
}

ExitConditionsBuilder& ExitConditionsBuilder::add_tolerance(double linear_tolerance,
                                              double angular_tolerance,
                                              double tolerance_time) {
    auto ec = std::make_shared<ToleranceExitCondition>();
    ec->add_lin_exit(linear_tolerance, tolerance_time);
    ec->add_ang_exit(angular_tolerance, tolerance_time);
    ec_ptr->conditions.push_back(
        std::dynamic_pointer_cast<AbstractExitCondition>(ec));
    return *this;
}

ExitConditionsBuilder& ExitConditionsBuilder::add_thru_smoothness(double smoothness) {
    ec_ptr->conditions.push_back(
        std::make_shared<PrepLineExitCondition>(smoothness));
    return *this;
}

ExitConditionsBuilder&
ExitConditionsBuilder::add_custom_condition(std::function<bool()> callback) {
    ec_ptr->conditions.push_back(std::make_shared<CustomExitCondition>(callback));
    return *this;
}

ExitConditionsBuilder&
ExitConditionsBuilder::add_condition(std::shared_ptr<AbstractExitCondition> ec) {
    ec_ptr->conditions.push_back(ec);
    return *this;
}
std::shared_ptr<ExitConditions> ExitConditionsBuilder::build() {
    return ec_ptr;
}
};