
#include "PPController.hpp"
#include "VOSS/utils/angle.hpp"

namespace voss::controller {
chassis::DiffChassisCommand
PPController::get_command(bool reverse, bool thru,
                          std::shared_ptr<AbstractExitCondition> ec) {
    double leftVel, rightVel;
    int dir = reverse ? -1 : 1;
    Point current_pos = this->l->get_position();
    double h = this->l->get_orientation_rad();
    Point closest_pt = this->target_path.at(this->get_closest());
    Point lookahead_pt = this->get_lookahead_pt().value_or(prev_lookahead_pt);

    double curvature = this->get_curvature(this->l->get_pose(), lookahead_pt);



    if (ec->is_met(this->l->get_pose(), thru)) {
        if (thru) {
        }
        return chassis::Stop{};
    }
    return chassis::DiffChassisCommand{
        voss::chassis::diff_commands::Voltages{leftVel, rightVel}};
}

chassis::DiffChassisCommand
PPController::get_angular_command(bool reverse, bool thru,
                                  voss::AngularDirection direction,
                                  std::shared_ptr<AbstractExitCondition> ec) {
    return chassis::Stop{};
}

std::vector<Point> PPController::inject_points() {
    double spacing = 6.0;
    std::vector<Point> new_path;
    for (int i = 0; i < this->target_path.size() - 1; i++) {
        Point pt0 = target_path.at(i);
        Point pt1 = target_path.at(i + 1);
        double mag = Point::getDistance(pt1, pt0);
        int num_pts = ceil(mag / spacing);
        Point normal_vec = {(pt1.x - pt0.x) / mag, (pt1.y - pt0.y) / mag};
        for (int p = 0; p < num_pts; p++) {
            new_path.push_back(
                {pt0.x + normal_vec.x * p, pt0.y + normal_vec.y * p});
        }
    }
    return new_path;
}

void PPController::process_path() {
    std::vector<Point> new_path = inject_points(); // inject nodes

    double change = tolerance;
    while (change >= tolerance) {
        change = 0.0;
        for (int i = 1; i < this->target_path.size() - 1; i++) { // modify x
            double aux = new_path[i].x;
            new_path[i].x +=
                {this->weight_data * (this->target_path[i].x - new_path[i].x) +
                 weight_smooth * (new_path[i - 1].x + new_path[i + 1].x -
                                  (2.0 * new_path[i].x))};
            change += std::abs(aux - new_path[i].x);
        }
        for (int i = 1; i < this->target_path.size() - 1; i++) { // modify y
            double aux = new_path[i].y;
            new_path[i].y +=
                {this->weight_data * (this->target_path[i].y - new_path[i].y) +
                 weight_smooth * (new_path[i - 1].y + new_path[i + 1].y -
                                  (2.0 * new_path[i].y))};
            change += std::abs(aux - new_path[i].y);
        }
    }

    this->target_path = new_path;
}

void PPController::get_distances_each_pt() {
    distances.push_back(0);
    distance_along_path.push_back(0);
    for (int i = 0; i < this->target_path.size(); i++) {
        Point pt0 = target_path.at(i - 1);
        Point pt1 = target_path.at(i);
        double distance = Point::getDistance(pt0, pt1);
        distance_along_path.push_back(distance + distance_along_path.at(i - 1));
        distances.push_back(distance);
    }
}

double PPController::get_curvature(const Point& P, const Point& Q,
                                   const Point& R) {

    double dx = P.x - Q.x;
    if (dx == 0) {
        dx = 0.0001;
    }
    double x1 = P.x;
    double y1 = P.y;
    double x2 = Q.x;
    double y2 = Q.y;
    double x3 = R.x;
    double y3 = R.y;

    double k1 = 0.5 * (x1 * x1 + y1 * y1 - x2 * x2 - y2 * y2) / (x1 - x2);
    double k2 = (y1 - y2) / (x1 - x2);
    double b =
        0.5 *
        (x2 * x2 - 2 * x2 * k1 + y2 * y2 - x3 * x3 + 2 * x3 * k1 - y3 * y3) /
        (x3 * k2 - y3 + y2 - x2 * k2);
    double a = k1 - k2 * b;
    double r = std::sqrt((x1 - a) * (x1 - a) + (y1 - b) + (y1 - b));
    if (r == 0) {
        return 0;
    }
    double curvature = 1.0 / r;

    return curvature;
}

void PPController::get_curvature() {
    curvatures.push_back(0);
    for (int i = 1; i < this->target_path.size() - 1; i++) {
        curvatures.push_back(this->get_curvature(this->target_path[i - 1],
                                                 this->target_path[i],
                                                 this->target_path[i + 1]));
    }
    curvatures.push_back(0);
}

void PPController::generate_profile() {
    for (int i = 0; i < this->target_path.size(); i++) {
        // Set up initial value for each point's velocity via curvature
        profile.push_back(std::fmin(100, chase_power / curvatures.at(i)));
    }

    for (int i = this->target_path.size() - 1; i >= 0; i--) {
        if (i == this->target_path.size() - 1) {
            profile.at(i) = {0.0};
        } else {
            double distance = distances.at(i + 1);
            double nextVelocity = profile.at(i + 1);
            double calculatedSpeed =
                std::sqrt(nextVelocity * nextVelocity +
                          2.0 * this->max_accelration * distance);

            profile.at(i) = std::fmin(profile.at(i), calculatedSpeed);
        }
    }
}

int PPController::get_closest() {
    int index = -1;
    double closestDist = -1;

    for (int i = this->prev_closest_index; i < this->target_path.size(); i++) {
        Point pt = this->target_path.at(i);
        double checkDist = Point::getDistance(pt, this->l->get_position());

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

std::optional<Point> PPController::get_lookahead_pt() {
    for (int i = this->target_path.size() - 1; i > this->prev_lookahead_index;
         i--) {
        Point last = this->target_path.at(i - 1);
        Point cur = this->target_path.at(i);

        Point intersect = circleLineIntersect(last, cur);

        if (intersect != this->l->get_position()) {
            return Point{intersect.x, intersect.y};
        }
    }
    return std::nullopt;
}

Point PPController::circleLineIntersect(Point p1, Point p2) {
    Point d = p2 - p1;
    Point f = p1 - this->l->get_position();
    double a = d * d;
    double b = 2 * (f * d);
    double c = (f * f) - pow(this->lookahead_dist, 2);
    double discriminant = pow(b, 2) - 4 * a * c;

    if (discriminant >= 0) {
        discriminant = sqrt(discriminant);
        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);

        double t = -1;
        if (t2 >= 0 && t2 <= 1)
            t = t2;
        if (t1 >= 0 && t1 <= 1)
            t = t1;
        if (t == -1)
            return this->l->get_position();

        return {std::lerp(p1.x, p2.x, t), std::lerp(p1.y, p2.y, t)};
    }
    return this->l->get_position();
}

void PPController::reset() {
    this->firstIter = true;
    this->distance_along_path.clear();
    this->curvatures.clear();
    this->distances.clear();
    this->profile.clear();
    this->target_path.clear();
    this->prev_closest_index = 0;
}
double PPController::get_curvature(const Pose& robot, const Point& pt) {
    const double a = -tan(robot.theta.value());
    const double b = 1;
    const double c = tan(robot.theta.value()) * robot.x - robot.y;

    double x = abs(pt.x * a + pt.y * b + c) / sqrt(a * a + b * b);
    double sideL = sin(robot.theta.value()) * (pt.x - robot.x) -
                   cos(robot.theta.value()) * (pt.y - pt.y);
    int side = sgn(sideL);

    if (sideL == 0) {
        return 0;
    }

    double chord = Point::getDistance({robot.x, robot.y}, pt);

    return (2 * x) / (chord * chord) * side;
}

} // namespace voss::controller