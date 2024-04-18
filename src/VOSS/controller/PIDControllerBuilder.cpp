#include "VOSS/controller/PIDControllerBuilder.hpp"

#include "VOSS/controller/PIDController.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/utils/angle.hpp"
#include <utility>

namespace voss::controller {

PIDControllerBuilder::PIDControllerBuilder(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : ctrl(std::move(l)) {

    this->ctrl.p = nullptr;
}

PIDControllerBuilder PIDControllerBuilder::new_builder(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
    PIDControllerBuilder builder(std::move(l));
    return builder;
}

PIDControllerBuilder PIDControllerBuilder::from(PIDController pid) {
    PIDControllerBuilder builder(nullptr);
    builder.ctrl = pid;
    return builder;
}

PIDControllerBuilder&
PIDControllerBuilder::with_linear_constants(double kP, double kI, double kD) {
    this->ctrl.linear_pid.set_constants(kP, kI, kD);
    return *this;
}

PIDControllerBuilder&
PIDControllerBuilder::with_angular_constants(double kP, double kI, double kD) {
    this->ctrl.angular_pid.set_constants(kP, kI, kD);
    return *this;
}

PIDControllerBuilder& PIDControllerBuilder::with_min_error(double error) {
    this->ctrl.min_error = error;
    return *this;
}

PIDControllerBuilder&
PIDControllerBuilder::with_min_vel_for_thru(double min_vel) {
    this->ctrl.min_vel = min_vel;
    return *this;
}

std::shared_ptr<PIDController> PIDControllerBuilder::build() {
    return std::make_shared<PIDController>(this->ctrl);
}

} // namespace voss::controller