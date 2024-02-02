#include "VOSS/controller/SwingControllerBuilder.hpp"
#include "VOSS/controller/SwingController.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/utils/angle.hpp"

namespace voss::controller {

SwingControllerBuilder::SwingControllerBuilder(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : ctrl(l) {
}

SwingControllerBuilder SwingControllerBuilder::new_builder(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
    SwingControllerBuilder builder(l);
    return builder;
}

SwingControllerBuilder&
SwingControllerBuilder::with_angular_constants(double kP, double kI,
                                               double kD) {
    this->ctrl.angular_kP = kP;
    this->ctrl.angular_kI = kI;
    this->ctrl.angular_kD = kD;
    return *this;
}

SwingControllerBuilder&
SwingControllerBuilder::with_angular_exit_error(double error) {
    this->ctrl.angular_exit_error = voss::to_radians(error);
    return *this;
}

SwingControllerBuilder& SwingControllerBuilder::with_settle_time(double time) {
    this->ctrl.settle_time = (time > 0) ? time : 250;
    return *this;
}

SwingController SwingControllerBuilder::build() {
    return this->ctrl;
}

} // namespace voss::controller