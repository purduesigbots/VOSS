#include "VOSS/controller/BoomerangController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>

namespace voss::controller {

BoomerangController::BoomerangController(
    std::shared_ptr<localizer::AbstractLocalizer> l)
    : AbstractController(l) {
}

chassis::ChassisCommand BoomerangController::get_command(bool reverse,
                                                         bool thru) {
    // TODO: finish

    Point current_pos = this->l->get_position();
    double k = this->min_vel / 100;
    Point virtualTarget = {target.x + k * cos(voss::to_radians(target.theta)),
                           target.y + k * sin(voss::to_radians(target.theta))};
    double dx = virtualTarget.x - current_pos.x;
    double dy = virtualTarget.y - current_pos.y;
    double distance_error = sqrt(dx * dx + dy * dy);
    double at = voss::to_radians(target.theta);
    this->carrotPoint = {virtualTarget.x - distance_error * cos(at) * lead_pct,
                         virtualTarget.y - distance_error * sin(at) * lead_pct,
                         target.theta};
    counter += 10;
    int dir = reverse ? -1 : 1;

    double current_angle = this->l->get_orientation_rad();
    bool chainedExecutable = false;
    bool noPose = this->target.theta == 361;
    double angle_error;
    double m = fabs((current_pos.y - target.y) / (current_pos.x - target.x));

    angle_error = atan2(dy, dx) - current_angle;

    if (reverse) {
        angle_error = atan2(-dy, -dx) - current_angle;
    }

    angle_error = voss::norm_delta(angle_error);

    double lin_speed;

    if (thru &&
        current_pos.y + this->min_error >=
            (-1.0 / m) * (current_pos.x + min_error - virtualTarget.x) + virtualTarget.y
        /*thru && (current_pos.y - virtualTarget.y) *
                cos(voss::to_radians(target.theta) + M_PI_2) <
            (current_pos.x - virtualTarget.x) *
                sin(voss::to_radians(target.theta) + M_PI_2) &&
        (pow((current_pos.y - virtualTarget.y), 2) +
         pow((current_pos.x - virtualTarget.x), 2)) < pow(10, 2)*/) {
        chainedExecutable = true;
    }

    if (distance_error <= exit_error ||
        (distance_error < min_error && fabs(cos(angle_error)) <= 0.1)) {
        total_lin_err = 0;
        close += 10;
    } else {
        close = 0;
    }

    if (close > settle_time) {
        return chassis::ChassisCommand{chassis::Stop{}};
    }

    if (fabs(distance_error - prev_lin_err) < 0.1 &&
        fabs(current_angle - prev_angle) < voss::to_radians(0.1) &&
        counter > 400) {
        close_2 += 10;
    } else {
        close_2 = 0;
    }

    prev_angle = current_angle;

    if (close_2 > settle_time * 2) {
        return chassis::ChassisCommand{chassis::Stop{}};
    }

    lin_speed = dir * linear_pid(distance_error);
    if (thru) {
        lin_speed = fmax(lin_speed, this->min_vel);
    }

    double ang_speed;
    if (distance_error < min_error && !thru) {
        this->can_reverse = true;

        if (noPose) {
            ang_speed = 0; // disable turning when close to the point to prevent
                           // spinning
        } else {
            // turn to face the finale pose angle if executing a pose movement
            double poseError = (target.theta * M_PI / 180) - current_angle;
            while (fabs(poseError) > M_PI)
                poseError -= 2 * M_PI * poseError / fabs(poseError);
            ang_speed = angular_pid(poseError);
        }

        // reduce the linear speed if the bot is tangent to the target
        lin_speed *= cos(angle_error);

    } else {
        if (fabs(angle_error) > M_PI_2 && this->can_reverse) {
            angle_error =
                angle_error - (angle_error / fabs(angle_error)) * M_PI;
            lin_speed = -lin_speed;
        }

        ang_speed = angular_pid(angle_error);
    }

    if (thru) {
        lin_speed *= cos(angle_error);
    }

    if (chainedExecutable) {
        return chassis::ChassisCommand{chassis::Chained{
            dir * std::fmax(lin_speed, this->min_vel) - ang_speed,
            dir * std::fmax(lin_speed, this->min_vel) + ang_speed}};
    }

    return chassis::ChassisCommand{
        chassis::Voltages{lin_speed - ang_speed, lin_speed + ang_speed}};
}

chassis::ChassisCommand BoomerangController::get_angular_command(bool reverse,
                                                                 bool thru) {
    return chassis::ChassisCommand{chassis::Stop{}};
}

double BoomerangController::linear_pid(double error) {
    total_lin_err += error;

    this->vel = error - prev_ang_err;

    double speed = linear_kP * error + linear_kD * (error - prev_lin_err) +
                   linear_kI * total_lin_err;

    this->prev_lin_err = error;

    return speed;
}

double BoomerangController::angular_pid(double error) {
    total_ang_err += error;

    double speed = angular_kP * error + angular_kD * (error - prev_ang_err) +
                   angular_kI * total_ang_err;

    this->prev_ang_err = error;

    return speed;
}

void BoomerangController::reset() {
    this->prev_lin_err = 0;
    this->total_lin_err = 0;
    this->prev_ang_err = 0;
    this->total_ang_err = 0;
    this->can_reverse = false;
    this->counter = 0;
    this->min_vel = 0;
}

} // namespace voss::controller