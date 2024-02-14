#pragma once

#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/utils/flags.hpp"

namespace voss::controller {

class PIDController : public AbstractController {
  protected:
    double linear_kP, linear_kI, linear_kD;
    double angular_kP, angular_kI, angular_kD;
    double tracking_kP;
    double exit_error;
    double angular_exit_error;
    double min_error;
    double can_reverse;
    double settle_time;

    double close;
    double close_2;
    int counter;
    double prev_angle;
    bool turn_overshoot;

    double prev_lin_err, total_lin_err, prev_ang_err, total_ang_err;

  public:
    PIDController(std::shared_ptr<localizer::AbstractLocalizer> l);

    double linear_pid(double error);
    double angular_pid(double error);

    chassis::ChassisCommand get_command(bool reverse, bool thru) override;
    chassis::ChassisCommand
    get_angular_command(bool reverse, bool thru,
                        voss::AngularDirection direction) override;

    void reset() override;

    friend class PIDControllerBuilder;
    friend class BoomerangControllerBuilder;
};

} // namespace voss::controller