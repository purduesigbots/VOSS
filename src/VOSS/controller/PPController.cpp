
#include "PPController.hpp"
#include "VOSS/utils/Math.hpp"

namespace voss::controller {
chassis::DiffChassisCommand
PPController::get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                          bool reverse, bool thru,
                          std::shared_ptr<AbstractExitCondition> ec) {
    double left_vel, right_vel;
    Pose current_pose = l->get_pose();
    Point look_ahead_pt;

    // v_i = = velocity at prior point, a = max acceleration, d = distance
    // between the points v_f = sqrt(v_i * v_i + 2 * a * d)


    double curvature = this->get_curvature(l->get_pose(), look_ahead_pt);
    double vel;
    double accel;
    // L = V * (2 + CT)/2
    // R = V * (2 − CT)/2

    if(reverse) {
        curvature *= -1;
        vel *= -1;
        accel *= -1;
    }


    left_vel = vel * (2.0 + curvature * this->track_width) / 2.0;
    right_vel = vel * (2.0 - curvature * this->track_width) / 2.0;

    left_vel = left_ffwd.update(left_vel, (left_vel - prev_left_vel)); //change these to mp accel?
    right_vel = left_ffwd.update(left_vel, (left_vel - prev_left_vel));

    left_vel += left_pid.update((left_vel - prev_left_vel)); // it would be nice if we can get actual vel
    right_vel += left_pid.update((left_vel - prev_left_vel)); // we would probably want to get actual vel from localizer


    if (!ec->is_met(l->get_pose(),
                    thru)) { // we might not want thru for pp but who knows
        return chassis::diff_commands::Voltages{left_vel, right_vel};
    }
    return chassis::Stop();
}

chassis::DiffChassisCommand PPController::get_angular_command(
    std::shared_ptr<localizer::AbstractLocalizer> l, bool reverse, bool thru,
    voss::AngularDirection direction,
    std::shared_ptr<AbstractExitCondition> ec) {
    return voss::chassis::Stop();
}

void PPController::reset() {
}

std::shared_ptr<PPController> PPController::get_ptr() {
    return this->shared_from_this();
}

int PPController::get_closest(const Point& current_pos) {
    int index = -1;
    double closestDist = -1;

    for (int i = this->prev_closest_index; i < this->target_path.size(); i++) {
        auto pose = this->target_path.at(i);
        double checkDist = Point::getDistance({pose.x, pose.y}, current_pos);

        if (index == -1) {
            index = i;
            closestDist = checkDist;
        } else {
            if (checkDist <= closestDist) {
                index = i;
                closestDist = checkDist;
            }
        }
    }

    this->prev_closest_index = index;
    return index;
}
double PPController::get_curvature(const Pose& robot, const Point& pt) const {
    const Point current_pt = {robot.x, robot.y};
    const double h = robot.theta.value();

    double a = -tan(h);
    double b = 1.;
    double c = tan(h) * current_pt.x - current_pt.y;

    double x = fabs(a * pt.x + b * pt.y + c) / sqrt(a * a + b * b);

    double side = sgn(sin(h) * (pt.x - robot.x) - cos(h) * (pt.y - robot.y));
    if(side == 0) {
        return 0;
    }

    return side * (2.0 * x / look_ahead_dist * look_ahead_dist); // side * curvature
}
std::optional<Point> PPController::get_lookahead_pt() {
    
}

}; // namespace voss::controller
