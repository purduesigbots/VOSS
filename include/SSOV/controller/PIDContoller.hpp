#pragma once

namespace ssov {
    struct PIDConstants {
        double kP;
        double kI;
        double kD;
    };

    class PIDController {
        private:
            const PIDConstants gain;
            double prev_error;
            double total_error;
            uint32_t prev_time;
        public:
            PIDController(PIDConstants constants): gain(constants), prev_error(0), total_error(0) {};
            double update(double error);
            void reset();
            PIDConstants get_constants() {
                return gain;
            }
    };
}