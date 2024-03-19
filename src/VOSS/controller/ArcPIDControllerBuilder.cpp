#include "VOSS/controller/ArcPIDControllerBuilder.hpp"
#include "ArcPIDController.hpp"

#include <utility>

namespace voss::controller {

ArcPIDControllerBuilder::ArcPIDControllerBuilder(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : ctrl(std::move(l)) {

    this->ctrl.p = nullptr;
}

ArcPIDControllerBuilder ArcPIDControllerBuilder::new_builder(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
    ArcPIDControllerBuilder builder(std::move(l));
    return builder;
}

ArcPIDControllerBuilder ArcPIDControllerBuilder::from(ArcPIDController arc) {
    ArcPIDControllerBuilder builder(arc.l);
    builder.ctrl = arc;
    return builder;
}

ArcPIDControllerBuilder&
ArcPIDControllerBuilder::with_linear_constants(double kP, double kI,
                                               double kD) {
    this->ctrl.linear_kP = kP;
    this->ctrl.linear_kI = kI;
    this->ctrl.linear_kD = kD;
    return *this;
}

ArcPIDControllerBuilder&
ArcPIDControllerBuilder::with_track_width(double track_width) {
    this->ctrl.track_width = track_width;
    return *this;
}

ArcPIDControllerBuilder& ArcPIDControllerBuilder::with_min_error(double error) {
    this->ctrl.min_error = error;
    return *this;
}

ArcPIDControllerBuilder& ArcPIDControllerBuilder::with_slew(double slew) {
    this->ctrl.slew = slew;
    return *this;
}

std::shared_ptr<ArcPIDController> ArcPIDControllerBuilder::build() {
    return std::make_shared<ArcPIDController>(this->ctrl);
}

} // namespace voss::controller