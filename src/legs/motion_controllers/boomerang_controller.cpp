#include "legs/motion_controllers/boomerang_controller.hpp"

#include "api.h"
#include "Eigen/Geometry"

namespace legs {

void BoomerangController::move(std::vector<double> target, double lead,
                               double max, double exit_error, MoveFlags flags) {
mode = TRANSLATIONAL;
linear_target[0] = target[0];
linear_target[1] = target[1];
angular_target =
    target.size() == 3 ? fmod(target.at(2), 360) : 361; // sentinel value

if (flags & RELATIVE) {
    double angle = model->getHeading() * M_PI / 180;
    Eigen::Rotation2Dd rotation_matrix(angle);
    linear_target = model->getPosition() + rotation_matrix * linear_target;
    if (target.size() == 3)
        angular_target += fmod(model->getHeading(), 360);
}

lead_pct = lead;
max_speed = max;
linear_exit_error = exit_error;
thru = flags & THRU;
reverse = flags & REVERSE;
can_reverse = false;

if (!(flags & ASYNC)) {
    waitUntilFinished();
    mode = DISABLE;
    if (!(flags & THRU)) {
        chassis->setAngularVelocity(0);
        chassis->setForwardVelocity(0);
        chassis->setHorizontalVelocity(0);
    }
}

}

void BoomerangController::move(std::vector<double> target, MoveFlags flags) {
    move(target, lead_pct, max_speed, linear_exit_error, flags);
}

void BoomerangController::turn(double target) {
}

void BoomerangController::waitUntilFinished() {
    pros::delay(400);
    switch (mode) {
        case TRANSLATIONAL:
            while ((model->getPose() - linear_target).squaredNorm() > linear_exit_error) {
                pros::delay(10);
            }

            if (angular_target != 361) {
                while (fabs(model->getHeading() - angular_target)) {
                    pros::delay(10);
                }
            }
            
            break;
        case ANGULAR:
            while (fabs(model->getHeading() - angular_target)) {
                    pros::delay(10);
            }
            break;
    }
}

BoomerangControllerBuilder::BoomerangControllerBuilder() {
}

BoomerangControllerBuilder& BoomerangControllerBuilder::withChassis(BasicChassis& chassis) {
    this->controller.chassis = std::shared_ptr<BasicChassis>(&chassis);
    return *this;
}

BoomerangControllerBuilder& BoomerangControllerBuilder::withModel(BasicModel& model) {
    this->controller.model = std::shared_ptr<BasicModel>(&model);
    return *this;
}

BoomerangControllerBuilder& BoomerangControllerBuilder::withLinearPid(double p, double i, double d) {
    this->controller.linear_pid = Pid(p,i,d);
    return *this;
}

BoomerangControllerBuilder& BoomerangControllerBuilder::withAngularPid(double p, double i, double d) {
    this->controller.angular_pid = Pid(p,i,d);
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