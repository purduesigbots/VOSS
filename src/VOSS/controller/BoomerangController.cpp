#include "VOSS/controller/BoomerangController.hpp"
#include "BoomerangControllerBuilder.hpp"
#include "PIDController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>

namespace voss::controller {

BoomerangController::BoomerangController(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : AbstractController(l) {
    child = std::make_shared<PIDController>(l);
}

chassis::ChassisCommand BoomerangController::get_command(bool reverse,
                                                         bool thru) {
    // TODO: finish
    if (target.theta > 360) {
        child->set_target(target, false);
    } else {
        Point current_pos = this->l->get_position();
        double dx = target.x - current_pos.x;
        double dy = target.y - current_pos.y;
        double distance_error = sqrt(dx * dx + dy * dy);
        double at = voss::to_radians(target.theta);
        Pose carrotPoint = {target.x - distance_error * cos(at) * lead_pct,
                            target.y - distance_error * sin(at) * lead_pct,
                            target.theta};
        // printf("current: %f %f\n", current_pos.x, current_pos.y);
        // printf("carrotPoint: %f %f\n", carrotPoint.x, carrotPoint.y);
        child->set_target(carrotPoint, false);
    }
    return child->get_command(reverse, thru);
}

chassis::ChassisCommand BoomerangController::get_angular_command(bool reverse,
                                                                 bool thru) {
    child->set_target(target, false);
    child->set_angular_target(angular_target, false);
    return child->get_angular_command(reverse, thru);
}

void BoomerangController::reset() {
    child->reset();
}

std::shared_ptr<BoomerangController>
BoomerangController::modify_linear_constants(double kP, double kI, double kD) {
    auto pid_mod = BoomerangControllerBuilder::from(*this)
                       .with_linear_constants(kP, kI, kD)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<BoomerangController>
BoomerangController::modify_angular_constants(double kP, double kI, double kD) {
    auto pid_mod = BoomerangControllerBuilder::from(*this)
                       .with_angular_constants(kP, kI, kD)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<BoomerangController>
BoomerangController::modify_tracking_kp(double kP) {
    auto pid_mod =
        BoomerangControllerBuilder::from(*this).with_tracking_kp(kP).build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<BoomerangController>
BoomerangController::modify_exit_error(double exit_error) {
    auto pid_mod = BoomerangControllerBuilder::from(*this)
                       .with_exit_error(exit_error)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<BoomerangController>
BoomerangController::modify_angular_exit_error(double exit_error) {
    auto pid_mod = BoomerangControllerBuilder::from(*this)
                       .with_angular_exit_error(exit_error)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<BoomerangController>
BoomerangController::modify_min_error(double min_error) {
    auto pid_mod = BoomerangControllerBuilder::from(*this)
                       .with_min_error(min_error)
                       .build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<BoomerangController>
BoomerangController::modify_lead_pct(double lead_pct) {
    auto pid_mod =
        BoomerangControllerBuilder::from(*this).with_lead_pct(lead_pct).build();

    this->p = pid_mod;

    return this->p;
}

std::shared_ptr<BoomerangController>
BoomerangController::modify_settle_time(double settle_time) {
    auto pid_mod = BoomerangControllerBuilder::from(*this)
                       .with_settle_time(settle_time)
                       .build();

    this->p = pid_mod;

    return this->p;
}

} // namespace voss::controller