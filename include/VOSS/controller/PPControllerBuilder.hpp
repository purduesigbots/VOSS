#pragma once

#include "VOSS/controller/PPController.hpp"

namespace voss::controller {
class PPControllerBuilder {
  private:
    controller::PPController ctrl;

  public:
    PPControllerBuilder(std::shared_ptr<localizer::AbstractLocalizer> l);

    static PPControllerBuilder
    new_builder(std::shared_ptr<localizer::AbstractLocalizer> l);

    PPControllerBuilder& with_linear_constants(double kP, double kI, double kD);
    PPControllerBuilder& with_angular_constants(double kP, double kI,
                                                double kD);
    PPControllerBuilder& with_tracking_kp(double kP);
    PPControllerBuilder& with_exit_error(double error);
    PPControllerBuilder& with_angular_exit_error(double error);
    PPControllerBuilder& with_min_error(double error);
    PPControllerBuilder& with_settle_time(double time);

    PPControllerBuilder& with_lookahead_distance(double lookahead);

    std::shared_ptr<PPController> build();
};
} // namespace voss::controller
