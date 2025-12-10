#pragma once

#include <vector>

#include "SSOV/trajectory/MotionState.hpp"

namespace ssov {

class MotionProfile {
    private:
        std::vector<MotionState> samples;
        std::vector<double> times;
    public:
        MotionProfile(std::vector<MotionState> samples);
        MotionProfile() = default;

        MotionState at(double t) const;
        double duration() const {
            return times.back();
        }
};

}