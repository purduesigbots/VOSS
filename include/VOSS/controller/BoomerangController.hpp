#pragma once

#include "AbstractController.hpp"
#include "PIDController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/utils/flags.hpp"

namespace voss::controller {

class BoomerangController : public AbstractController {
  protected:
    std::shared_ptr<BoomerangController> p;
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

    chassis::DiffChassisCommand get_command(bool reverse, bool thru) override;
    chassis::DiffChassisCommand
    get_angular_command(bool reverse, bool thru,
                        voss::AngularDirection direction) override;

    double linear_pid(double error);
    double angular_pid(double error);

    void reset() override;

    std::shared_ptr<BoomerangController>
    modify_linear_constants(double kP, double kI, double kD);
    std::shared_ptr<BoomerangController>
    modify_angular_constants(double kP, double kI, double kD);
    std::shared_ptr<BoomerangController> modify_exit_error(double error);
    std::shared_ptr<BoomerangController>
    modify_angular_exit_error(double error);
    std::shared_ptr<BoomerangController> modify_min_error(double error);
    std::shared_ptr<BoomerangController> modify_lead_pct(double lead_pct);
    std::shared_ptr<BoomerangController> modify_settle_time(double time);

    friend class BoomerangControllerBuilder;
};

} // namespace voss::controller