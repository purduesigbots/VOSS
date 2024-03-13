#pragma once

#include "VOSS/controller/AbstractController.hpp"
#include <memory>

namespace voss::controller {

class ArcPIDController : public AbstractController {
  protected:
    std::shared_ptr<ArcPIDController> p;
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

    chassis::DiffChassisCommand get_command(bool reverse, bool thru) override;
    chassis::DiffChassisCommand
    get_angular_command(bool reverse, bool thru,
                        voss::AngularDirection direction) override;

    void reset() override;

    std::shared_ptr<ArcPIDController>
    modify_linear_constants(double kP, double kI, double kD);
    std::shared_ptr<ArcPIDController> modify_track_width(double track_width);
    std::shared_ptr<ArcPIDController> modify_exit_error(double error);
    std::shared_ptr<ArcPIDController> modify_min_error(double error);
    std::shared_ptr<ArcPIDController> modify_settle_time(double time);
    std::shared_ptr<ArcPIDController> modify_slew(double slew);

    friend class ArcPIDControllerBuilder;
};

} // namespace voss::controller