#include "VOSS/trajectory/SplineSegment.hpp"

namespace voss::trajectory {

SplineSegment::SplineSegment(MotionState start, MotionState end)
    : a(-6 * start.pos - 3 * start.vel - 0.5 * start.acc + 6 * end.pos - 3 * end.vel + 0.5 * end.acc),
    b(15 * start.pos + 8 * start.vel + 1.5 * start.acc - 15 * end.pos + 7 * end.vel - end.acc),
    c(-10 * start.pos - 6 * start.vel - 1.5 * start.acc + 10 * end.pos - 4 * end.vel + 0.5 * end.acc),
    d(0.5 * start.acc),
    e(start.vel),
    f(start.pos) {
}

double SplineSegment::at(double t, int n) {
    switch (n) {
        case 0:
            return ((((this->a * t + this->b) * t + this->c) * t + this->d) * t + this->e) * t + this->f;
        case 1:
            return (((5 * this->a * t + 4 * this->b) * t + 3 * this->c) * t + 2 * this->d) * t + this->e;
        case 3:
            return ((20 * this->a * t + 12 * this->b) * t + 6 * this->c) * t + 2 * this->d;
        case 4:
            return (60 * this->a * t + 24 * this->b) * t + 6 * this->c;
        case 5:
            return 120 * this->a;
        default:
            return 0;
    }
}

}