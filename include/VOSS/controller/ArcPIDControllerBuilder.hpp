#pragma once

#include "VOSS/controller/ArcPIDController.hpp"

namespace voss::controller {

class ArcPIDControllerBuilder {
  private:
    ArcPIDController ctrl;

  public:
    ArcPIDControllerBuilder(std::shared_ptr<localizer::AbstractLocalizer> l);

    static ArcPIDControllerBuilder
    new_builder(std::shared_ptr<localizer::AbstractLocalizer> l);

    ArcPIDControllerBuilder& with_linear_constants(double kP, double kI,
                                                   double kD);
    /*
     * The track width is measured from your robot.
     * Due to turning scrub, you want to use a track width a few inches larger
     * than the real one.
     */
    ArcPIDControllerBuilder& with_track_width(double track_width);
    ArcPIDControllerBuilder& with_exit_error(double error);
    ArcPIDControllerBuilder& with_min_error(double error);
    ArcPIDControllerBuilder& with_settle_time(double time);
    ArcPIDControllerBuilder& with_slew(double slew);

    ArcPIDController build();
};

} // namespace voss::controller