#pragma once

#include "SSOV/trajectory/MotionState.hpp"

namespace ssov {

class SplineSegment {
    private:
        double a, b, c, d, e, f;
    public:
        SplineSegment(MotionState start, MotionState end)
            : a(-6 * start.pos - 3 * start.vel - 0.5 * start.acc + 6 * end.pos - 3 * end.vel + 0.5 * end.acc),
            b(15 * start.pos + 8 * start.vel + 1.5 * start.acc - 15 * end.pos + 7 * end.vel - end.acc),
            c(-10 * start.pos - 6 * start.vel - 1.5 * start.acc + 10 * end.pos - 4 * end.vel + 0.5 * end.acc),
            d(0.5 * start.acc),
            e(start.vel),
            f(start.pos) {};
        MotionState at(double t) const {
            double pos = ((((a * t + b) * t + c) * t + d) * t + e) * t + f;
            double vel = (((5 * a * t + 4 * b) * t + 3 * c) * t + 2 * d) * t + e;
            double acc = ((20 * a * t + 12 * b) * t + 6 * c) * t + 2 * d;
            return {pos, vel, acc};
        }
};

}