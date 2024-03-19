#include "VOSS/utils/PID.hpp"

namespace voss::utils {

PID::PID(double kP, double kI, double kD): kP(kP), kI(kI), kD(kD), prev_error(0.0), total_error(0.0) {
}

double PID::update(double error) {
    total_error += error;

    double output = kP * error + kI * total_error + kD * (error - prev_error);

    prev_error = error;

    return output;
}

void PID::reset() {
    total_error = 0.0;
    prev_error = 0.0;
}

}