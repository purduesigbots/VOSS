
#include "VOSS/utils/FeedFwd.hpp"
#include "VOSS/utils/Math.hpp"

namespace voss::utils {

FeedForward::FeedForward(double kS, double kV, double kA, double kD)
    : kS(kS), kD(kD), kA(kA), kV(kV){};

double FeedForward::update(double velocity, double acceleration) {
    return kS * sgn(velocity) + kV * velocity +
           (acceleration > 0 ? kA : kD) * acceleration;
}

void FeedForward::set_constants(double kS, double kV, double kA, double kD) {
    this->kS = kS;
    this->kV = kV;
    this->kA = kA;
    this->kD = kD;
}

} // namespace voss