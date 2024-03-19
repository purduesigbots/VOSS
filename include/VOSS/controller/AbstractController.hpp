#pragma once

#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/utils/flags.hpp"

namespace voss::controller {

class AbstractController {

  protected:
    std::shared_ptr<localizer::AbstractLocalizer> l;
    Pose target;
    double angular_target;

  public:
    AbstractController(std::shared_ptr<localizer::AbstractLocalizer> l);

    virtual chassis::DiffChassisCommand
    get_command(bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) = 0;
    virtual chassis::DiffChassisCommand
    get_angular_command(bool reverse, bool thru,
                        voss::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) = 0;

    virtual void reset() = 0;

    void set_target(Pose target, bool relative,
                    std::shared_ptr<AbstractExitCondition> ec);
    void set_angular_target(double angle, bool relative);
};

} // namespace voss::controller