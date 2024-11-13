#pragma once

#include "SSOV/common/Pose.hpp"

namespace ssov {

struct TrajectoryConstraints {
    double max_vel;
    double max_accel;
    double max_decel;
    double max_ang_accel;
    double max_centr_accel;
    double track_width;
};

struct TrajectoryState {
    Pose pose;
    Pose vel;
};

class Trajectory {
    public:
        virtual TrajectoryState at(double t) = 0;
        virtual double duration() = 0;
};

}