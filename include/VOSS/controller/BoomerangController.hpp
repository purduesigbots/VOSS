#pragma once

#include "AbstractController.hpp"
#include "PIDController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"

namespace voss::controller {

class BoomerangController : public AbstractController {
  protected:
    double lead_pct;
    Pose carrotPoint;

    double linear_kP, linear_kI, linear_kD;
    double angular_kP, angular_kI, angular_kD;
    double vel;
    double exit_error;
    double angular_exit_error;
    double min_error;
    bool can_reverse;
    double settle_time;

    double close;
    double close_2;
    int counter;
    double prev_angle;
    double min_vel;

    double prev_lin_err, total_lin_err, prev_ang_err, total_ang_err;

  public:
    BoomerangController(std::shared_ptr<localizer::AbstractLocalizer> l);

    chassis::ChassisCommand get_command(bool reverse, bool thru) override;
    chassis::ChassisCommand get_angular_command(bool reverse,
                                                bool thru) override;

    double linear_pid(double error);
    double angular_pid(double error);

    void reset();

    friend class BoomerangControllerBuilder;
};

} // namespace voss::controller