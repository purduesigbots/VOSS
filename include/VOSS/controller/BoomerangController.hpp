#pragma once

#include "AbstractController.hpp"
#include "PIDController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"

namespace voss::controller {

class BoomerangController : public AbstractController {
  protected:
    std::shared_ptr<PIDController> child = nullptr;
    double lead_pct;

  public:
    BoomerangController(std::shared_ptr<localizer::AbstractLocalizer> l);

    chassis::ChassisCommand
    get_command(bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) override;
    chassis::ChassisCommand
    get_angular_command(bool reverse, bool thru,
                        std::shared_ptr<AbstractExitCondition> ec) override;

    void reset() override;

    friend class BoomerangControllerBuilder;
};

} // namespace voss::controller