#include "SSOV/trajectory/QuinticSplinePath.hpp"

#include "SSOV/common/Math.hpp"

namespace ssov {

QuinticSplinePath::QuinticSplinePath(std::initializer_list<Pose> waypoints, bool reversed): Path(reversed) {
    for (auto waypoint = waypoints.begin() + 1; waypoint != waypoints.end(); waypoint++) {
        Pose prev = *(waypoint - 1);
        MotionState begin_x, end_x, begin_y, end_y;
        begin_x.pos = prev.x;
        begin_y.pos = prev.y;
        end_x.pos = waypoint->x;
        end_y.pos = waypoint->y;
        double dist = distance(prev.to_point(), waypoint->to_point());
        if (reversed) {
            dist *= -1;
        }
        begin_x.vel = cos(prev.theta) * dist;
        begin_y.vel = sin(prev.theta) * dist;
        end_x.vel = cos(waypoint->theta) * dist;
        end_y.vel = sin(waypoint->theta) * dist;
        begin_x.acc = 0;
        begin_y.acc = 0;
        end_x.acc = 0;
        end_y.acc = 0;

        this->x_segments.push_back(SplineSegment(begin_x, end_x));
        this->y_segments.push_back(SplineSegment(begin_y, end_y));
    }

    const auto f = [this](double t) {
        int index = t;
        if (index >= num_segments()) {
            index = num_segments() - 1;
        }
        double dx = x_segments[index].at(t - index).vel;
        double dy = y_segments[index].at(t - index).vel;
        return distance({dx, dy}, {0, 0});
    };

    arc_length_lookup = IntegralScan(0.0, this->num_segments(), 1E-6, f);
}

PoseWithCurvature QuinticSplinePath::at(double distance) const {
    double t = arc_length_lookup.lookup_inverse(distance);
    int index = t;
    if (index >= num_segments()) {
        index = num_segments() - 1;
    }

    MotionState x_state = x_segments[index].at(t - index);
    MotionState y_state = y_segments[index].at(t - index);

    PoseWithCurvature result;
    result.pose.x = x_state.pos;
    result.pose.y = y_state.pos;
    result.pose.theta = atan2(y_state.vel, x_state.vel);
    if (is_reversed()) {
        result.pose.theta += M_PI;
    }
    result.pose.theta = norm(result.pose.theta);
    double d1_d2_cross = x_state.vel * y_state.acc - y_state.vel * x_state.acc;
    double d1_norm = ssov::distance({x_state.vel, y_state.vel}, {0, 0});
    result.curvature = d1_d2_cross / (d1_norm * d1_norm * d1_norm);
    return result;
}

}