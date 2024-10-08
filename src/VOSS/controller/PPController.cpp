
#include "PPController.hpp"
#include "../../../include/VOSS/controller/PPController.hpp"
#include "VOSS/utils/angle.hpp"
#include "VOSS/utils/Math.hpp"

namespace voss::controller {

PPController::PPController(PPController::Params params)
    : linear_pid(params.lin_kp, params.lin_ki, params.lin_kd),
      angular_pid(params.ang_kp, params.ang_ki, params.ang_kd),
      look_ahead_dist(params.look_ahead_dist), track_width(params.track_width) {
}

//controller::PPController::PPController(double lin_kp, double lin_ki,
//                                       double lin_kd, double ang_kp,
//                                       double ang_ki, double ang_kd,
//                                       double look_ahead_dist,
//                                       double track_width)
//    : linear_pid(lin_kp, lin_ki, lin_kd), angular_pid(ang_kp, ang_ki, ang_kd),
//      look_ahead_dist(look_ahead_dist), track_width(track_width) {
//}

std::shared_ptr<PPController>
PPController::create_controller(PPController::Params params) {
    return std::move(std::make_shared<PPController>(params));
}

chassis::DiffChassisCommand
PPController::get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                          std::shared_ptr<AbstractExitCondition> ec,
                          const velocity_pair& v_pair, bool reverse,
                          bool thru) {
    double left_vel, right_vel;
    Point current_pos = l->get_position();
    int dir = reverse ? -1 : 1;

    int idx = this->get_closest(l->get_pose());
    printf("idx: %d\n", idx);

    Point look_ahead_pt = this->getLookAheadPoint(l->get_pose());
    printf("x: %lf, y: %lf\n", look_ahead_pt.x, look_ahead_pt.y);
    if (Point::getDistance(look_ahead_pt, {this->target_path.back().x,
                                           this->target_path.back().y}) <
            this->look_ahead_dist &&
        idx == target_path.size() - 1) {
        const Pose target = target_path.back();

        auto vel_pair =
            pid_controller_for_ending(l->get_pose(), target, reverse);
        left_vel = vel_pair.first;
        right_vel = vel_pair.second;

    } else {

        auto err_mat = get_relative_error(l->get_pose(), look_ahead_pt);

        double lin_vel = linear_pid.update(err_mat.x / this->look_ahead_dist) *
                         (reverse ? -1 : 1);

        double ang_vel = angular_pid.update(err_mat.y / look_ahead_dist);

        double curvature = this->get_curvature(l->get_pose(), look_ahead_pt);
        // L = V * (2 + CT)/2
        // R = V * (2 − CT)/2
        left_vel = dir * (lin_vel - ang_vel);
        right_vel = dir * (lin_vel + ang_vel);

        double vMax = std::max(fabs(left_vel), fabs(right_vel));
        if (vMax > 100) {
            left_vel *= 100.0 / vMax;
            right_vel *= 100.0 / vMax;
        }
    }

    if (!ec->is_met(l->get_pose(),
                    thru)) { // we might not want thru for pp but who knows
        return chassis::diff_commands::Voltages{left_vel, right_vel};
    }
    return chassis::Stop();
}

std::pair<double, double>
PPController::pid_controller_for_ending(const Pose& current_pos,
                                        const Pose& target, bool reverse) {
    int dir = reverse ? -1 : 1;
    double current_angle = current_pos.theta.value();
    bool noPose = !target.theta.has_value();

    double dx = target.x - current_pos.x;
    double dy = target.y - current_pos.y;

    double distance_error = sqrt(dx * dx + dy * dy);

    double angle_error = atan2(dy, dx) - current_angle;

    if (reverse) {
        angle_error = atan2(-dy, -dx) - current_angle;
    }

    angle_error = voss::norm_delta(angle_error);

    double lin_speed = linear_pid.update(distance_error) * dir;

    double ang_speed;
    if (distance_error < min_error) {
        this->can_reverse = true;


        if (noPose) {
            ang_speed = 0; // disable turning when close to the point to prevent
                           // spinning
        } else {
            // turn to face the finale pose angle if executing a pose movement
            double poseError = target.theta.value() - current_angle;

            while (fabs(poseError) > M_PI)
                poseError -= 2 * M_PI * poseError / fabs(poseError);
            ang_speed = angular_pid.update(poseError);
        }

        // reduce the linear speed if the bot is tangent to the target
        lin_speed *= cos(angle_error);

    } else if (distance_error < 2 * min_error) {
        // scale angular speed down to 0 as distance_error approaches min_error
        ang_speed = angular_pid.update(angle_error);
        ang_speed *= (distance_error - min_error) / min_error;
    } else {
        if (fabs(angle_error) > M_PI_2 && this->can_reverse) {
            angle_error =
                angle_error - (angle_error / fabs(angle_error)) * M_PI;
            lin_speed = -lin_speed;
        }

        ang_speed = angular_pid.update(angle_error);
    }
    lin_speed = std::max(-100.0, std::min(100.0, lin_speed));
    return {lin_speed - ang_speed, lin_speed + ang_speed};
}

int PPController::get_closest(const Pose& current_pos) {
    double dx = target_path.front().x - current_pos.x;
    double dy = target_path.front().y - current_pos.y;
    double min_dist = sqrt(dx * dx + dy * dy);
    int minIndex = 0;
    for (int i = 1; i < this->target_path.size(); i++) {
        dx = this->target_path.at(i).x - current_pos.x;
        dy = this->target_path.at(i).y - current_pos.y;
        double dist = sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            minIndex = i;
        }
    }
    return minIndex;
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


// calculating the intersect of the circle and the line, don't mess with the math because it comes directly from the paper
// also I don't know how it works
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

Point PPController::get_relative_error(const Pose& robot_pt, const Point& pt) {
    double dx = pt.x - robot_pt.x;
    double dy = pt.y - robot_pt.y;

    double h = robot_pt.theta.value();
    float r_x = dy * cos(h) - dx * sin(h);
    float r_y = dy * sin(h) + dx * cos(h);

    return {r_x, r_y};
}

Point PPController::getLookAheadPoint(const Pose& robot) {
    double dx = this->target_path.back().x - robot.x;
    double dy = this->target_path.back().y - robot.y;
    double dist = sqrt(dx * dx + dy * dy);
    if (dist < this->look_ahead_dist) {
        return {this->target_path.back().x, this->target_path.back().y};
    }
    int closest_idx = this->get_closest(robot);
    voss::Point look_ahead_pt(0, 0);
    Point robot_position = {robot.x, robot.y};

    for (int i = 1; i < this->target_path.size(); i++) {
        voss::Pose cur_pt = this->target_path.at(i);
        voss::Pose prev_pt = this->target_path.at(i + 1);

        // if suitable distance is found
        if (voss::Point::getDistance({cur_pt.x, cur_pt.y}, robot_position) >
                this->look_ahead_dist &&
            voss::Point::getDistance({prev_pt.x, prev_pt.y}, robot_position) <
                this->look_ahead_dist &&
            closest_idx < i) {

            // interpolation
            double prevX = prev_pt.x;
            double prevY = prev_pt.y;

            double currX = cur_pt.x;
            double currY = cur_pt.y;

            double minT = 0;
            double maxT = 1;

            double newX = prevX;
            double newY = prevY;

            int iterations = 10;

            // binary approximation
            for (int z = 0; z < iterations; z++) {
                double midT = (minT + maxT) / 2.0;
                newX = prevX * (1 - midT) + currX * midT;
                newY = prevY * (1 - midT) + currY * midT;

                look_ahead_pt = {newX, newY};

                double distToSelf =
                    voss::Point::getDistance(look_ahead_pt, robot_position);

                if (distToSelf < look_ahead_dist) {
                    minT = midT;
                } else {
                    maxT = midT;
                }
            }
            return look_ahead_pt;
        }
    }

    look_ahead_pt = {this->target_path[closest_idx].x,
                     this->target_path[closest_idx].y};
    return look_ahead_pt;
}

void PPController::reset() {
    angular_pid.reset();
    linear_pid.reset();
}

}; // namespace voss::controller
