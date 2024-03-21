#include "VOSS/exit_conditions/ExitConditions.hpp"
#include "VOSS/exit_conditions/CustomExitCondition.hpp"
#include "VOSS/exit_conditions/PrepLineExitCondition.hpp"
#include "VOSS/exit_conditions/SettleExitCondition.hpp"
#include "VOSS/exit_conditions/TimeOutExitCondition.hpp"
#include "VOSS/exit_conditions/ToleranceAngularExitCondition.hpp"
#include "VOSS/exit_conditions/ToleranceExitCondition.hpp"
#include "VOSS/exit_conditions/ToleranceLinearExitCondition.hpp"
#include "VOSS/utils/Pose.hpp"
#include <memory>

namespace voss::controller {

ExitConditions::ExitConditions() {
}

ExitConditions ExitConditions::new_conditions() {
    ExitConditions ec;

    return ec;
}

void ExitConditions::set_target(voss::Pose new_target) {
    this->target_pose = new_target;

    for (auto ec : this->conditions) {
        ec->set_target(this->target_pose);
    }
}

ExitConditions& ExitConditions::add_settle(int settle_time, double tolerance,
                                           int initial_delay) {
    SettleExitCondition ec(settle_time, tolerance, initial_delay);
    this->conditions.push_back(std::make_shared<SettleExitCondition>(ec));
    return *this;
}

ExitConditions& ExitConditions::add_timeout(int timeout) {
    this->conditions.push_back(std::make_shared<TimeOutExitCondition>(timeout));
    return *this;
}

ExitConditions& ExitConditions::add_tolerance(double linear_tolerance,
                                              double angular_tolerance,
                                              double tolerance_time) {
    auto ec = std::make_shared<ToleranceExitCondition>();
    ec->add_lin_exit(linear_tolerance, tolerance_time);
    ec->add_ang_exit(angular_tolerance, tolerance_time);
    this->conditions.push_back(
        std::dynamic_pointer_cast<AbstractExitCondition>(ec));
    return *this;
}

ExitConditions& ExitConditions::add_thru_smoothness(double smoothness) {
    this->conditions.push_back(
        std::make_shared<PrepLineExitCondition>(smoothness));
    return *this;
}

ExitConditions&
ExitConditions::add_custom_condition(std::function<bool()> callback) {
    this->conditions.push_back(std::make_shared<CustomExitCondition>(callback));
    return *this;
}

std::shared_ptr<ExitConditions>
ExitConditions::exit_if(std::function<bool()> callback) {
    std::shared_ptr<ExitConditions> ec_mod =
        std::make_shared<ExitConditions>(*this);
    ec_mod->conditions.push_back(
        std::make_shared<CustomExitCondition>(callback));
    return ec_mod;
}

ExitConditions&
ExitConditions::add_condition(std::shared_ptr<AbstractExitCondition> ec) {
    this->conditions.push_back(ec);
    return *this;
}

bool ExitConditions::is_met(voss::Pose current_pose, bool thru) {
    for (auto ec : this->conditions) {
        if (ec->is_met(current_pose, thru)) {
            return true;
        }
    }

    return false;
}

bool ExitConditions::all_met(voss::Pose current_pose, bool thru) {
    for (auto ec : this->conditions) {
        if (!ec->is_met(current_pose, thru)) {
            return false;
        }
    }

    return true;
}

std::shared_ptr<ExitConditions> ExitConditions::build() {
    return std::make_shared<ExitConditions>(*this);
}

void ExitConditions::reset() {
    for (auto ec : this->conditions) {
        ec->reset();
    }
}

} // namespace voss::controller