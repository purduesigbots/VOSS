#pragma once

#include "pros/rtos.hpp"

namespace ssov {
// Uses a PI loop to estimate position and velocity
// from noisy position data.
class TrackingLoopEstimator {
    private:
        const double kP;
        const double kI;
        double pos_est = 0;
        double vel_est = 0;
        double vel_int = 0;
        uint32_t prev_time;
    public:
        TrackingLoopEstimator(double kP, double kI): kP(kP), kI(kI) {};
        void reset(double pos) {
            pos_est = pos;
            vel_est = 0;
            vel_int = 0;
            prev_time = pros::millis();
        }
        std::array<double, 3> update(double pos_meas) {
            uint32_t current_time = pros::millis();
            double dt = (current_time - prev_time) / 1000.0;
            prev_time = current_time;
            pos_est += vel_est * dt;
            double pos_err = pos_meas - pos_est;
            vel_int += pos_err * kI * dt;
            vel_est = pos_err * kP + vel_int;
            return {pos_est, vel_est, vel_int};
        }
};

}