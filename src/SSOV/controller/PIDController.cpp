#include "SSOV/controller/PIDContoller.hpp"

#include "pros/rtos.hpp"

namespace ssov {

double PIDController::update(double error) {
    uint32_t current_time = pros::millis();
    double dt = 0.001 * (current_time - prev_time); // convert time to seconds
    total_error += 0.5 * (prev_error + error) * dt; // trapezoidal integration
    double output = gain.kP * error + gain.kI * total_error + gain.kD * (error - prev_error) / dt;
    prev_error = error;
    prev_time = current_time;
    return output;
}

void PIDController::reset() {
    total_error = 0;
    prev_error = 0;
    prev_time = pros::millis();
}

}