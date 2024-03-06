#include "CRPIDControllerBuilder.hpp"
#include "VOSS/utils/angle.hpp"

namespace voss::controller {
CRPIDControllerBuilder::CRPIDControllerBuilder(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : ctrl(std::move(l)) {
}

CRPIDControllerBuilder CRPIDControllerBuilder::new_builder(
    std::shared_ptr<localizer::AbstractLocalizer> l) {
    CRPIDControllerBuilder builder(std::move(l));
    return builder;
}

CRPIDControllerBuilder&
CRPIDControllerBuilder::with_linear_constants(double kP, double kI, double kD) {
    this->ctrl.linear_kP = kP;
    this->ctrl.linear_kI = kI;
    this->ctrl.linear_kD = kD;
    return *this;
}

CRPIDControllerBuilder&
CRPIDControllerBuilder::with_angular_constants(double kP, double kI, double kD) {
    this->ctrl.angular_kP = kP;
    this->ctrl.angular_kI = kI;
    this->ctrl.angular_kD = kD;
    return *this;
}

CRPIDControllerBuilder& CRPIDControllerBuilder::with_r(double r) {
    this->ctrl.r = r;
    return *this;
}

CRPIDControllerBuilder& CRPIDControllerBuilder::with_exit_error(double error) {
    this->ctrl.exit_error = error;
    return *this;
}

CRPIDControllerBuilder&
CRPIDControllerBuilder::with_angular_exit_error(double error) {
    this->ctrl.angular_exit_error = voss::to_radians(error);
    return *this;
}

CRPIDControllerBuilder& CRPIDControllerBuilder::with_min_error(double error) {
    this->ctrl.min_error = error;
    return *this;
}

CRPIDControllerBuilder& CRPIDControllerBuilder::with_settle_time(double time) {
    this->ctrl.settle_time = (time > 0) ? time : 250;
    return *this;
}

std::shared_ptr<CRPIDController> CRPIDControllerBuilder::build() {
    return std::make_shared<CRPIDController>(this->ctrl);
}
}