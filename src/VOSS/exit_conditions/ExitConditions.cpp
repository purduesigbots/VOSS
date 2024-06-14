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

void ExitConditions::set_target(voss::Pose new_target) {
    this->target_pose = new_target;

    for (auto ec : this->conditions) {
        ec->set_target(this->target_pose);
    }
}

std::shared_ptr<ExitConditions> ExitConditions::set_settle(int settle_time,
                                                           double tolerance,
                                                           int initial_delay) {
    auto ec = std::make_shared<SettleExitCondition>(settle_time, tolerance,
                                                    initial_delay);
    auto opt = this->ec_is_repeated<SettleExitCondition>();
    this->p = std::make_shared<ExitConditions>(*this);
    if (opt.has_value()) {
        this->p->conditions.at(opt.value()) = std::move(ec);
    } else {
        this->p->conditions.push_back(ec);
    }

    return this->p;
}

std::shared_ptr<ExitConditions> ExitConditions::set_timeout(int timeout) {
    auto ec = std::make_shared<TimeOutExitCondition>(timeout);
    auto opt = this->ec_is_repeated<TimeOutExitCondition>();
    this->p = std::make_shared<ExitConditions>(*this);
    if (opt.has_value()) {
        this->p->conditions.at(opt.value()) = std::move(ec);
    } else {
        this->p->conditions.push_back(ec);
    }

    return this->p;
}

std::shared_ptr<ExitConditions>
ExitConditions::set_tolerance(double linear_tolerance, double angular_tolerance,
                              double tolerance_time) {
    auto ec = std::make_shared<ToleranceExitCondition>();
    ec->add_lin_exit(linear_tolerance, tolerance_time);
    ec->add_ang_exit(angular_tolerance, tolerance_time);

    auto opt = this->ec_is_repeated<ToleranceExitCondition>();
    this->p = std::make_shared<ExitConditions>(*this);
    if (opt.has_value()) {
        this->p->conditions.at(opt.value()) = std::move(ec);
    } else {
        this->p->conditions.push_back(ec);
    }

    return this->p;
}

std::shared_ptr<ExitConditions>
ExitConditions::set_linear_tolerance(double linear_tolerance,
                                     double tolerance_time) {
    this->p = std::make_shared<ExitConditions>(*this);
    auto opt = this->ec_is_repeated<ToleranceExitCondition>();
    if (opt.has_value()) {
        std::dynamic_pointer_cast<ToleranceExitCondition>(
            this->p->conditions.at(opt.value()))
            ->add_lin_exit(linear_tolerance, tolerance_time);
    } else {
        this->p->set_tolerance(linear_tolerance, 0, tolerance_time);
    }
    return this->p;
}

std::shared_ptr<ExitConditions>
ExitConditions::set_angular_tolerance(double angular_tolerance,
                                      double tolerance_time) {
    this->p = std::make_shared<ExitConditions>(*this);
    auto opt = this->ec_is_repeated<ToleranceExitCondition>();
    if (opt.has_value()) {
        std::dynamic_pointer_cast<ToleranceExitCondition>(
            this->p->conditions.at(opt.value()))
            ->add_ang_exit(angular_tolerance, tolerance_time);
    } else {
        this->p->set_tolerance(0, angular_tolerance, tolerance_time);
    }
    return this->p;
}

std::shared_ptr<ExitConditions>
ExitConditions::set_thru_smoothness(double smoothness) {
    auto ec = std::make_shared<PrepLineExitCondition>(smoothness);

    auto opt = this->ec_is_repeated<PrepLineExitCondition>();
    this->p = std::make_shared<ExitConditions>(*this);
    if (opt.has_value()) {
        this->p->conditions.at(opt.value()) = std::move(ec);
    } else {
        this->p->conditions.push_back(ec);
    }

    return this->p;
}

std::shared_ptr<ExitConditions>
ExitConditions::exit_if(std::function<bool()> callback) {
    this->p = std::make_shared<ExitConditions>(*this);
    this->p->conditions.push_back(
        std::make_shared<CustomExitCondition>(callback));
    return p;
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

void ExitConditions::reset() {
    for (auto ec : this->conditions) {
        ec->reset();
    }
}

} // namespace voss::controller