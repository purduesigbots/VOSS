#include "VOSS/trajectory/SplinePath.hpp"

#include <cmath>
#include <functional>
#include "VOSS/utils/Point.hpp"

namespace voss::trajectory {

SplinePath::SplinePath(std::initializer_list<Pose> waypoints) {
    for (auto waypoint = waypoints.begin() + 1; waypoint != waypoints.end(); waypoint++) {
        Pose prev = *(waypoint - 1);
        MotionState begin_x, end_x, begin_y, end_y;
        begin_x.pos = prev.x;
        begin_y.pos = prev.y;
        end_x.pos = waypoint->x;
        end_y.pos = waypoint->y;
        double dist = voss::Point::getDistance({prev.x, prev.y}, {waypoint->x, waypoint->y});
        begin_x.vel = cos(prev.theta.value()) * dist;
        begin_y.vel = sin(prev.theta.value()) * dist;
        end_x.vel = cos(waypoint->theta.value()) * dist;
        end_y.vel = sin(waypoint->theta.value()) * dist;
        begin_x.acc = 0;
        begin_y.acc = 0;
        end_x.acc = 0;
        end_y.acc = 0;

        this->x_segments.push_back(SplineSegment(begin_x, end_x));
        this->y_segments.push_back(SplineSegment(begin_y, end_y));
    }

    auto f = [this](double t) {
        int index = t;
        double dx = x_segments[index].at(t - index).vel;
        double dy = y_segments[index].at(t - index).vel;
        return voss::Point::getDistance({dx, dy}, {0, 0});
    };

    arc_length_reparam = voss::utils::IntegralScan(0.0, this->length(), 1E-6, f);
}

SplinePath::SplinePath(std::vector<Pose> waypoints) {
    for (auto waypoint = waypoints.begin() + 1; waypoint != waypoints.end(); waypoint++) {
        Pose prev = *(waypoint - 1);
        MotionState begin_x, end_x, begin_y, end_y;
        begin_x.pos = prev.x;
        begin_y.pos = prev.y;
        end_x.pos = waypoint->x;
        end_y.pos = waypoint->y;
        double dist = voss::Point::getDistance({prev.x, prev.y}, {waypoint->x, waypoint->y});
        begin_x.vel = cos(prev.theta.value()) * dist;
        begin_y.vel = sin(prev.theta.value()) * dist;
        end_x.vel = cos(waypoint->theta.value()) * dist;
        end_y.vel = sin(waypoint->theta.value()) * dist;
        begin_x.acc = 0;
        begin_y.acc = 0;
        end_x.acc = 0;
        end_y.acc = 0;

        this->x_segments.push_back(SplineSegment(begin_x, end_x));
        this->y_segments.push_back(SplineSegment(begin_y, end_y));
    }

    auto f = [this](double t) {
        int index = t;
        double dx = x_segments[index].at(t - index).vel;
        double dy = y_segments[index].at(t - index).vel;
        return voss::Point::getDistance({dx, dy}, {0, 0});
    };

    arc_length_reparam = voss::utils::IntegralScan(0.0, this->length(), 1E-6, f);
}

double SplinePath::length() {
    return this->x_segments.size();
}

PoseWithCurvature SplinePath::at(double distance) {
    double t = arc_length_reparam.lookup_inverse(distance);
    int index = t;
    MotionState x_state = x_segments[index].at(t - index);
    MotionState y_state = y_segments[index].at(t - index);

    PoseWithCurvature result;
    result.pose.x = x_state.pos;
    result.pose.y = y_state.pos;
    result.pose.theta = atan2(y_state.vel, x_state.vel);
    double d1_d2_cross = x_state.vel * y_state.acc - y_state.vel * x_state.acc;
    double d1_norm = voss::Point::getDistance({x_state.vel, y_state.vel}, {0, 0});
    result.curvature = d1_d2_cross / (d1_norm * d1_norm * d1_norm);
    return result;
}

PathSample SplinePath::sample(double dist_resolution) {
    double arc_length = arc_length_reparam.end();
    int num_samples = std::max(1.0, ceil(arc_length / dist_resolution));

    PathSample result;
    for (int i = 0; i <= num_samples; i++) {
        double distance = arc_length / num_samples * i;
        PoseWithCurvature pose = this->at(distance);
        result.distances.push_back(distance);
        result.poses.push_back(pose);
    }

    return result;
}

}