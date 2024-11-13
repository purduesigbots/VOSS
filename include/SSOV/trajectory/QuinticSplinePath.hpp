#pragma once

#include <initializer_list>

#include "SSOV/trajectory/Path.hpp"
#include "SSOV/trajectory/QuinticSplineSegment.hpp"
#include "SSOV/trajectory/IntegralScan.hpp"

namespace ssov {

class QuinticSplinePath: public Path {
    private:
        std::vector<SplineSegment> x_segments;
        std::vector<SplineSegment> y_segments;
        IntegralScan arc_length_lookup;
    public:
        QuinticSplinePath(std::initializer_list<Pose> waypoints, bool reversed);
        double num_segments() const {
            return x_segments.size();
        }
        double length() const override {
            return arc_length_lookup.end();
        }
        PoseWithCurvature at(double distance) const override;
};

}