#pragma once

#include "VOSS/trajectory/MotionState.hpp"

namespace voss::trajectory {

class SplineSegment {
  private:
    double a, b, c, d, e, f;

  public:
    SplineSegment(MotionState start, MotionState end);

    double at(double t, int n);
};

}