#pragma once

#include "PIDController.hpp"
#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"

namespace voss::controller {
class SwingController : public AbstractController {
  protected:
    double angular_kP, angular_kI, angular_kD;
    double angular_exit_error;
    double settle_time;
    bool can_reverse;

    double close;
    double close_2;
    int counter;

    double prev_ang_err, total_ang_err;
    double prev_ang_speed;

  public:
    SwingController(std::shared_ptr<localizer::AbstractLocalizer> l);

    chassis::ChassisCommand
    get_command(bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) override;
    chassis::ChassisCommand
    get_angular_command(bool reverse, bool thru,
                        std::shared_ptr<AbstractExitCondition> ec) override;

    double angular_pid(double error);

    void reset() override;

    friend class SwingControllerBuilder;
};
} // namespace voss::controller
