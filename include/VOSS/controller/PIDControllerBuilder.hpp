#pragma once

#include "VOSS/controller/PIDController.hpp"

namespace voss::controller {

class PIDControllerBuilder {
  private:
    controller::PIDController ctrl;

  public:
    PIDControllerBuilder(std::shared_ptr<localizer::AbstractLocalizer> l);

    static PIDControllerBuilder
    new_builder(std::shared_ptr<localizer::AbstractLocalizer> l);

    PIDControllerBuilder& with_linear_constants(double kP, double kI,
                                                double kD);
    PIDControllerBuilder& with_angular_constants(double kP, double kI,
                                                 double kD);
    PIDControllerBuilder& with_tracking_kp(double kP);
    PIDControllerBuilder& with_exit_error(double error);
    PIDControllerBuilder& with_angular_exit_error(double error);
    PIDControllerBuilder& with_min_error(double error);
    PIDControllerBuilder& with_settle_time(double time);
    PIDControllerBuilder& with_min_vel_for_thru(double min_vel);

    PIDController build();
};

} // namespace voss::controller