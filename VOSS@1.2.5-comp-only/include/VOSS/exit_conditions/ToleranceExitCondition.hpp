
#pragma once
#include "AbstractExitCondition.hpp"
#include "ToleranceAngularExitCondition.hpp"
#include "ToleranceLinearExitCondition.hpp"
#include <memory>
namespace voss::controller {
class ToleranceExitCondition : public AbstractExitCondition {
  private:
    std::shared_ptr<ToleranceAngularExitCondition> ang_exit = nullptr;
    std::shared_ptr<ToleranceLinearExitCondition> lin_exit = nullptr;

  public:
    void set_target(voss::Pose target) override;
    bool is_met(Pose pose, bool thru) override;
    void reset() override;
    void add_ang_exit(double angular_tolerance, double tolerance_time);
    void add_lin_exit(double linear_tolerance, double tolerance_time);
};
} // namespace voss::controller
