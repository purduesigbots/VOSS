#include "VOSS/utils/Math.hpp"

namespace voss {
double linear_to_rotational(double velocity_inches_per_sec, double wheel_diameter, double gear_ratio) {
    return (velocity_inches_per_sec / (M_PI * wheel_diameter)) * 60.0 * gear_ratio;
}

} // namespace voss
