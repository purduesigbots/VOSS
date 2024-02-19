#include "VOSS/exit_conditions/ExitConditions.hpp"
#include "VOSS/exit_conditions/SettleExitCondition.hpp"
#include "VOSS/exit_conditions/TimeOutExitCondition.hpp"
#include "VOSS/exit_conditions/ToleranceAngularExitCondition.hpp"
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

ExitConditions& ExitConditions::set_target(voss::Pose new_target) {
    this->target_pose = new_target;

    for (auto ec : this->conditions) {
        ec->set_target(this->target_pose);
    }

    return *this;
}

ExitConditions& ExitConditions::add_settle(int settle_time, double tolerance) {
    SettleExitCondition ec(this->target_pose, settle_time, tolerance);
    this->conditions.push_back(std::make_shared<SettleExitCondition>(ec));
    return *this;
}

ExitConditions& ExitConditions::add_timeout(int timeout) {
    TimeOutExitCondition ec(timeout);
    this->conditions.push_back(std::make_shared<TimeOutExitCondition>(ec));
    return *this;
}

ExitConditions& ExitConditions::add_angular_tolerance(double tolerance) {
    ToleranceAngularExitCondition ec(this->target_pose, tolerance);
    this->conditions.push_back(
        std::make_shared<ToleranceAngularExitCondition>(ec));
    return *this;
}

ExitConditions& ExitConditions::add_linear_tolerance(double tolerance) {
    ToleranceLinearExitCondition ec(this->target_pose, tolerance);
    this->conditions.push_back(
        std::make_shared<ToleranceLinearExitCondition>(ec));
    return *this;
}

ExitConditions&
ExitConditions::add_condition(std::shared_ptr<AbstractExitCondition> ec) {
    this->conditions.push_back(ec);
    return *this;
}

bool ExitConditions::is_met(voss::Pose current_pose) {
    for (auto ec : this->conditions) {
        if (ec->is_met(current_pose)) {
            return true;
        }
    }

    return false;
}

bool ExitConditions::all_met(voss::Pose current_pose) {
    for (auto ec : this->conditions) {
        if (!ec->is_met(current_pose)) {
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