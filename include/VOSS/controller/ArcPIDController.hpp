#pragma once

#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"

namespace voss::controller {

class ArcPIDController : public AbstractController {
  protected:
    double linear_kP, linear_kI, linear_kD;
    double track_width;
    double exit_error;
    double min_error;
    double can_reverse;
    double prev_t;
    double slew;
    double prev_lin_speed;

    double close;

    double prev_lin_err, total_lin_err;

  public:
    ArcPIDController(std::shared_ptr<localizer::AbstractLocalizer> l);

    double linear_pid(double error);

    chassis::ChassisCommand
    get_command(bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) override;
    chassis::ChassisCommand
    get_angular_command(bool reverse, bool thru,
                        std::shared_ptr<AbstractExitCondition> ec) override;

    void reset() override;

    friend class ArcPIDControllerBuilder;
};

} // namespace voss::controller