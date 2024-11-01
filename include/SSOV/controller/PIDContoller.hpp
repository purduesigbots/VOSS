#pragma once

#include "pros/rtos.hpp"

namespace ssov {
    struct PIDConstants {
        double kP;
        double kI;
        double kD;
    };

    class PIDController {
        private:
        PIDConstants gain;
        double prev_error;
        double total_error;
        uint32_t prev_time;
        public:
        PIDController(PIDConstants constants): gain(constants), prev_error(0), total_error(0) {};
        double update(double error) {
            uint32_t current_time = pros::millis();
            double dt = 0.001 * (current_time - prev_time);
            total_error += 0.5 * (prev_error + error) * dt;
            double output = gain.kP * error + gain.kI * total_error + gain.kD * (error - prev_error) / dt;
            prev_error = error;
            prev_time = current_time;
            return output;
        }
        void reset() {
            total_error = 0;
            prev_error = 0;
            prev_time = pros::millis();
        }
    };
}