
#include "PPController.hpp"
#include "VOSS/utils/angle.hpp"
#include "VOSS/utils/Math.hpp"

namespace voss::controller {

PPController::PPController(PPController::PP_Construct_Params params)
    : linear_pid(params.lin_kp, params.lin_ki, params.lin_kd),
      angular_pid(params.ang_kp, params.ang_ki, params.ang_kd),
      look_ahead_dist(params.look_ahead_dist), track_width(params.track_width) {
}

chassis::DiffChassisCommand
PPController::get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                          bool reverse, bool thru,
                          std::shared_ptr<AbstractExitCondition> ec) {
    double left_vel, right_vel;
    Point current_pos = l->get_position();
    double h = l->get_orientation_rad();
    int idx = this->get_closest(l->get_pose());

    Point look_ahead_pt =
        get_lookahead_pt(l->get_pose(), idx)
            .value_or(Point{target_path.at(prev_closest_index).x,
                            target_path.at(prev_closest_index).y});

    double lin_err = Point::getDistance(current_pos, look_ahead_pt);

    double lin_vel = linear_pid.update(lin_err) * (reverse ? -1 : 1);
    double ang_vel =
        angular_pid.update(get_reference_y_err(l->get_pose(), look_ahead_pt));

    double curvature = this->get_curvature(l->get_pose(), look_ahead_pt);
    // L = V * (2 + CT)/2
    // R = V * (2 − CT)/2

    left_vel = lin_vel * (2.0 - curvature * this->track_width) / 2.0 - ang_vel;
    right_vel = lin_vel * (2.0 + curvature * this->track_width) / 2.0 + ang_vel;

    double vMax = std::max(fabs(left_vel), fabs(right_vel));
    if (vMax > 100) {
        left_vel *= 100.0 / vMax;
        right_vel *= 100.0 / vMax;
    }

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


int PPController::get_closest(const Pose& current_pos) {
    double closestDist = std::numeric_limits<double>::max();
    int closest = -1;
    Point pos = {current_pos.x, current_pos.y};

    // loop from the last closest point to one point past the lookahead
    for (int i = 0; i < target_path.size(); i++) {
        auto it = target_path.at(i);

        double distance = Point::getDistance(pos, {it.x, it.y});
        if (distance < closestDist) {
            closestDist = distance;
            closest = i;
        }
    }

    prev_closest_index = closest;
    return closest;
}

double PPController::get_curvature(const Pose& robot, const Point& pt) const {
    const Point current_pt = {robot.x, robot.y};
    const double h = robot.theta.value();

    double a = -tan(h);
    double b = 1.;
    double c = tan(h) * current_pt.x - current_pt.y;

    double x = fabs(a * pt.x + b * pt.y + c) / sqrt(a * a + b * b);

    double side = sgn(sin(h) * (pt.x - robot.x) - cos(h) * (pt.y - robot.y));
    if (side == 0) {
        return 0;
    }

    return side *
           (2.0 * x / look_ahead_dist * look_ahead_dist); // side * curvature
}

std::optional<Point> PPController::get_lookahead_pt(const Pose& robot_pt,
                                                    int idx) {
    Pose prev_pose = target_path.at(idx);
    Pose curr_pose = target_path.at(idx + 1);

    return circle_line_intersect(robot_pt, prev_pose, curr_pose);
}

std::optional<Point> PPController::circle_line_intersect(
    const Pose& robot_pt, const Pose& start_pose, const Pose& end_pose) const {
    Point end_pt = {end_pose.x, end_pose.y};
    Point start_pt = {start_pose.x, start_pose.y};
    Point d = end_pt - start_pt;
    Point f = start_pt - Point{robot_pt.x, robot_pt.y};
    double a = d * d;
    double b = 2 * (f * d);
    double c = (f * f) - this->look_ahead_dist * this->look_ahead_dist;
    double discriminant = b * b - 4 * a * c;

    if (discriminant >= 0) {
        discriminant = sqrt(discriminant);
        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);

        double t = NAN;
        if (t1 >= 0 && t1 <= 1) {
            t = t1;
        } else if (t2 >= 0 && t2 <= 1) {
            t = t2;
        } else {
            return std::nullopt;
        }

        return std::optional<Point>({std::lerp(start_pt.x, end_pt.x, t),
                                     std::lerp(start_pt.y, end_pt.y, t)});
    }
    return std::nullopt;
}
double PPController::get_reference_y_err(const Pose& robot_pt,
                                         const Point& pt) {
    double dx = pt.x - robot_pt.x;
    double dy = pt.y - robot_pt.y;

    return dy * sin(robot_pt.theta.value()) + dx * cos(robot_pt.theta.value());
}

void PPController::reset() {
    angular_pid.reset();
    linear_pid.reset();
}

std::shared_ptr<PPController> PPController::get_ptr() {
    return this->shared_from_this();
}

}; // namespace voss::controller
