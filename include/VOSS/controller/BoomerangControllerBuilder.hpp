#pragma once

#include "VOSS/controller/BoomerangController.hpp"

namespace voss::controller {

class BoomerangControllerBuilder {
  private:
    controller::BoomerangController ctrl;

  public:
    BoomerangControllerBuilder(std::shared_ptr<localizer::AbstractLocalizer> l);

    static BoomerangControllerBuilder
    new_builder(std::shared_ptr<localizer::AbstractLocalizer> l);

    BoomerangControllerBuilder& with_linear_constants(double kP, double kI,
                                                      double kD);
    BoomerangControllerBuilder& with_angular_constants(double kP, double kI,
                                                       double kD);
    BoomerangControllerBuilder& with_tracking_kp(double kP);
    BoomerangControllerBuilder& with_exit_error(double error);
    BoomerangControllerBuilder& with_angular_exit_error(double error);
    BoomerangControllerBuilder& with_min_error(double error);
    BoomerangControllerBuilder& with_lead_pct(double lead_pct);

    std::shared_ptr<BoomerangController> build();
};

} // namespace voss::controller