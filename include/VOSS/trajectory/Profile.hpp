#pragma once

#include <vector>

#include "VOSS/trajectory/MotionState.hpp"

namespace voss::trajectory {

class Profile {
  private:
    std::vector<MotionState> samples;
    std::vector<double> times;

  public:
    Profile(std::vector<MotionState> samples);
    Profile() = default;

    MotionState at(double t);
    double duration();
};

}