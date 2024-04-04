#pragma once

#include "AbstractController.hpp"
#include "PIDController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/utils/flags.hpp"
#include "VOSS/utils/PID.hpp"

namespace voss::controller {

class BoomerangController : public AbstractController {
  protected:
    std::shared_ptr<BoomerangController> p;
    double lead_pct;
    Pose carrotPoint;

    utils::PID linear_pid, angular_pid;
    double min_error;
    bool can_reverse;

    double min_vel;

  public:
    BoomerangController(std::shared_ptr<localizer::AbstractLocalizer> l);

    chassis::DiffChassisCommand
    get_command(bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) override;
    chassis::DiffChassisCommand
    get_angular_command(bool reverse, bool thru,
                        voss::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) override;

    void reset() override;

    std::shared_ptr<BoomerangController>
    modify_linear_constants(double kP, double kI, double kD);
    std::shared_ptr<BoomerangController>
    modify_angular_constants(double kP, double kI, double kD);
    std::shared_ptr<BoomerangController> modify_min_error(double error);
    std::shared_ptr<BoomerangController> modify_lead_pct(double lead_pct);

    friend class BoomerangControllerBuilder;
};

} // namespace voss::controller