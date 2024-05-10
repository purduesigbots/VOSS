#include "VOSS/trajectory/SplinePath.hpp"

#include <cmath>
#include <functional>
#include "VOSS/utils/Integration.hpp"
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
}

double SplinePath::length() {
    return this->x_segments.size();
}

PoseWithCurvature SplinePath::at(double t) {
    int index = t;
    double x = x_segments[index].at(t - index, 0);
    double y = y_segments[index].at(t - index, 0);
    double dx = x_segments[index].at(t - index, 1);
    double dy = y_segments[index].at(t - index, 1);
    double dx2 = x_segments[index].at(t - index, 2);
    double dy2 = y_segments[index].at(t - index, 2);

    PoseWithCurvature result;
    result.pose.x = x;
    result.pose.y = y;
    result.pose.theta = atan2(dy, dx);
    double d1_d2_cross = dx * dy2 - dy * dx2;
    double d1_norm = voss::Point::getDistance({dx, dy}, {0, 0});
    result.curvature = d1_d2_cross / (d1_norm * d1_norm * d1_norm);
    return result;
}

PathSample SplinePath::sample(double dist_resolution) {
    auto f = [this](double t) {
        int index = t;
        double dx = x_segments[index].at(t - index, 1);
        double dy = y_segments[index].at(t - index, 1);
        return voss::Point::getDistance({dx, dy}, {0, 0});
    };

    voss::utils::IntegralScan scan(0.0, this->length(), 1E-6, f);

    double arc_length = scan.end();
    int num_samples = std::max(1.0, ceil(arc_length / dist_resolution));

    PathSample result;
    for (int i = 0; i <= num_samples; i++) {
        double distance = arc_length / num_samples * i;
        PoseWithCurvature pose = this->at(scan.lookup_inverse(distance));
        result.distances.push_back(distance);
        result.poses.push_back(pose);
    }

    return result;
}

}