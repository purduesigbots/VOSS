#include "VOSS/controller/PPControllerBuilder.hpp"
#include "PPController.hpp"
#include <memory>

namespace voss::controller {
PPControllerBuilder::PPControllerBuilder(std::shared_ptr<localizer::AbstractLocalizer> l)
    : ctrl(l) {
    };

PPControllerBuilder PPControllerBuilder::new_builder(std::shared_ptr<localizer::AbstractLocalizer> l) {
    PPControllerBuilder builder(l);
    return builder;
}

PPControllerBuilder& PPControllerBuilder::with_linear_constants(double kP, double kI, double kD) {
    this->ctrl.pidController->linear_kP = kP;
    this->ctrl.pidController->linear_kI = kI;
    this->ctrl.pidController->linear_kD = kD;
    return *this;
}

PPControllerBuilder& PPControllerBuilder::with_angular_constants(double kP, double kI, double kD) {
    this->ctrl.pidController->angular_kP = kP;
    this->ctrl.pidController->angular_kI = kI;
    this->ctrl.pidController->angular_kD = kD;
    return *this;
}

PPControllerBuilder& PPControllerBuilder::with_tracking_kp(double kP) {
    this->ctrl.pidController->tracking_kP = kP;
    return *this;
}

PPControllerBuilder& PPControllerBuilder::with_exit_error(double error) {
    this->ctrl.pidController->exit_error = error;
    return *this;
}

PPControllerBuilder& PPControllerBuilder::with_angular_exit_error(double error) {
    this->ctrl.pidController->angular_exit_error = voss::to_radians(error);
    return *this;
}

PPControllerBuilder& PPControllerBuilder::with_min_error(double error) {
    this->ctrl.pidController->min_error = error;
    return *this;
}

PPControllerBuilder& PPControllerBuilder::with_settle_time(double time) {
    this->ctrl.pidController->settle_time = (time > 0) ? time : 250;
    return *this;
}

PPControllerBuilder& PPControllerBuilder::with_PID(std::shared_ptr<PIDController> pid) {
    this->ctrl.pidController = pid;
    return *this;
}

PPControllerBuilder& PPControllerBuilder::with_lookahead_distance(double lookahead) {
    this->ctrl.lookAheadDist = lookahead;
    return *this;
}

PPControllerBuilder& PPControllerBuilder::with_slew(double slew) {
    this->ctrl.slewStep = slew;
    return *this;
}

std::shared_ptr<PPController> PPControllerBuilder::build() {
    return std::make_shared<PPController>(this->ctrl);
};
}