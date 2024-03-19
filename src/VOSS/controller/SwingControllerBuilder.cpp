#include "VOSS/controller/SwingControllerBuilder.hpp"

#include "VOSS/controller/SwingController.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/utils/angle.hpp"
#include <utility>

namespace voss::controller {

SwingControllerBuilder::SwingControllerBuilder(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : ctrl(std::move(l)) {

    this->ctrl.p = nullptr;
}

SwingControllerBuilder SwingControllerBuilder::new_builder(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
    SwingControllerBuilder builder(std::move(l));
    return builder;
}

SwingControllerBuilder SwingControllerBuilder::from(SwingController swc) {
    SwingControllerBuilder builder(swc.l);
    builder.ctrl = swc;
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

std::shared_ptr<SwingController> SwingControllerBuilder::build() {
    return std::make_shared<SwingController>(this->ctrl);
}

} // namespace voss::controller