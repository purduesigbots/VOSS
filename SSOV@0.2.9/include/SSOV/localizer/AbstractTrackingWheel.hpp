#pragma once

namespace ssov {
    class AbstractTrackingWheel {
        protected:
            AbstractTrackingWheel(double tpi): tpi(tpi) {};
            double tpi;
            virtual double get_raw_position() = 0;
        public:
            double get_dist_travelled() {
                return get_raw_position() / tpi;
            }
            virtual void reset() = 0;
    };
}