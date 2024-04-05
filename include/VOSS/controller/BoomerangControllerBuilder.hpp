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

    static BoomerangControllerBuilder from(BoomerangController bmr);

    BoomerangControllerBuilder& with_linear_constants(double kP, double kI,
                                                      double kD);
    BoomerangControllerBuilder& with_angular_constants(double kP, double kI,
                                                       double kD);
    BoomerangControllerBuilder& with_lead_pct(double lead_pct);
    BoomerangControllerBuilder& with_min_error(double error);
    BoomerangControllerBuilder& with_min_vel_for_thru(double min_vel);

    std::shared_ptr<BoomerangController> build();
};

} // namespace voss::controller