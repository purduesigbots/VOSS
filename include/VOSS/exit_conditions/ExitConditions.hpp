#pragma once

#include "AbstractExitCondition.hpp"
#include "ToleranceExitCondition.hpp"
#include "VOSS/utils/Pose.hpp"
#include <functional>
#include <memory>
#include <vector>

namespace voss::controller {

class ExitConditions : public AbstractExitCondition {
    friend class ExitConditionsBuilder;

  public:
    std::shared_ptr<ExitConditions>
    set_settle(int settle_time, double tolerance, int initial_delay);
    std::shared_ptr<ExitConditions> set_timeout(int timeout);
    std::shared_ptr<ExitConditions> set_tolerance(double linear_tolerance,
                                                  double angular_tolerance,
                                                  double tolerance_time);

    std::shared_ptr<ExitConditions>
    set_linear_tolerance(double linear_tolerance, double tolerance_time);
    std::shared_ptr<ExitConditions>
    set_angular_tolerance(double angular_tolerance, double tolerance_time);
    std::shared_ptr<ExitConditions> set_thru_smoothness(double smoothness);

    template <class EC_TYPE>
        requires(std::is_base_of<AbstractExitCondition, EC_TYPE>::value)
    std::shared_ptr<ExitConditions> set_condition(EC_TYPE ec,
                                                  bool replace_old = false) {
        this->p = std::make_shared<ExitConditions>(*this);
        if (!replace_old) {
            p->conditions.push_back(std::make_shared<EC_TYPE>(ec));
            return p;
        }
        auto opt = this->ec_is_repeated<EC_TYPE>();
        if (opt.has_value()) {
            this->p->conditions.at(opt.value()) = std::move(ec);
        } else {
            this->p->conditions.push_back(ec);
        }

        return this->p;
    }

    std::shared_ptr<ExitConditions> exit_if(std::function<bool()> callback);

    void set_target(voss::Pose new_target) override;
    bool is_met(voss::Pose current_pose, bool thru) override;
    bool all_met(voss::Pose current_pose, bool thru);

    void reset() override;

  private:
    ExitConditions();
    template <class T> std::optional<size_t> ec_is_repeated() {
        auto it =
            std::find_if(conditions.cbegin(), conditions.cend(),
                         [](const std::shared_ptr<AbstractExitCondition> ec) {
                             return std::dynamic_pointer_cast<T>(ec) != nullptr;
                         });
        if (it == conditions.cend()) {
            return std::nullopt;
        }
        return std::distance(conditions.cbegin(), it);
    };

  private:
    std::shared_ptr<ExitConditions> p;
    std::vector<std::shared_ptr<controller::AbstractExitCondition>> conditions;
};

} // namespace voss::controller