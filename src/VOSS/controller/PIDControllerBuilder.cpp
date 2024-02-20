#include "VOSS/controller/PIDControllerBuilder.hpp"
#include "VOSS/controller/PIDController.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/utils/angle.hpp"

namespace voss::controller {

PIDControllerBuilder::PIDControllerBuilder(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : ctrl(l) {

    this->ctrl.p = nullptr;
}

PIDControllerBuilder PIDControllerBuilder::new_builder(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
    PIDControllerBuilder builder(l);
    return builder;
}

PIDControllerBuilder PIDControllerBuilder::from(PIDController pid) {
    PIDControllerBuilder builder(pid.l);
    builder.ctrl = pid;
    return builder;
}

PIDControllerBuilder&
PIDControllerBuilder::with_linear_constants(double kP, double kI, double kD) {
    this->ctrl.linear_kP = kP;
    this->ctrl.linear_kI = kI;
    this->ctrl.linear_kD = kD;
    return *this;
}

PIDControllerBuilder&
PIDControllerBuilder::with_angular_constants(double kP, double kI, double kD) {
    this->ctrl.angular_kP = kP;
    this->ctrl.angular_kI = kI;
    this->ctrl.angular_kD = kD;
    return *this;
}

PIDControllerBuilder& PIDControllerBuilder::with_tracking_kp(double kP) {
    this->ctrl.tracking_kP = kP;
    return *this;
}

PIDControllerBuilder& PIDControllerBuilder::with_exit_error(double error) {
    this->ctrl.exit_error = error;
    return *this;
}

PIDControllerBuilder&
PIDControllerBuilder::with_angular_exit_error(double error) {
    this->ctrl.angular_exit_error = voss::to_radians(error);
    return *this;
}

PIDControllerBuilder& PIDControllerBuilder::with_min_error(double error) {
    this->ctrl.min_error = error;
    return *this;
}

PIDControllerBuilder& PIDControllerBuilder::with_settle_time(double time) {
    this->ctrl.settle_time = (time > 0) ? time : 250;
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