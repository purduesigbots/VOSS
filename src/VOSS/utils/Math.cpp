#include "VOSS/utils/Math.hpp"

namespace voss {
double linear_vel_to_rpm(double velocity_inches_per_sec,
                         double wheel_diameter) {
    return (velocity_inches_per_sec / (M_PI * wheel_diameter)) * 60;
}
} // namespace voss
