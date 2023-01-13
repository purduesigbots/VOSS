#include "legs/motion_controllers/boomerang_controller.hpp"

#include "api.h"

namespace legs {

void BoomerangController::move(Eigen::Vector3d& target) {

}

void BoomerangController::turn(double target) {

}

BoomerangControllerBuilder::BoomerangControllerBuilder() {
}

BoomerangControllerBuilder& BoomerangControllerBuilder::withChassis(BasicChassis chassis) {
    this->controller.chassis = chassis;
    return *this;
}

BoomerangControllerBuilder& BoomerangControllerBuilder::withModel(BasicModel model) {
    this->controller.model = model;
    return *this;
}

BoomerangControllerBuilder& BoomerangControllerBuilder::withLinearPid(double p, double i, double d) {
    this->controller.linearPid = Pid(p,i,d);
    return *this;
}

BoomerangControllerBuilder& BoomerangControllerBuilder::withAngularPid(double p, double i, double d) {
    this->controller.angularPid = Pid(p,i,d);
    return *this;
}

BoomerangControllerBuilder& BoomerangControllerBuilder::withExitErrors(double linear, double angular) {
    this->controller.linear_exit_error = linear;
    this->controller.angular_exit_error = angular;
    return *this;
}

BoomerangControllerBuilder& BoomerangControllerBuilder::withSettleThresh(double linear, double angular) {
    this->controller.linear_settle_thresh = linear;
    this->controller.angular_settle_thresh = angular;
    return *this;
}

BoomerangControllerBuilder& BoomerangControllerBuilder::withSettleTime(double time) {
    this->controller.settle_time = time;
    return *this;
}

BoomerangController BoomerangControllerBuilder::build() {
    return this->controller;
}

}