#include "SSOV/localizer/OdometryLocalizer.hpp"

#include <cmath>

namespace ssov {
void OdometryLocalizer::update() {
    Pose local_change = get_local_change();
    double dtheta = local_change.theta;
    double s, c;
    if (abs(dtheta) < 1e-9) {
        s = 1 - dtheta * dtheta / 6;
        c = dtheta / 2;
    } else {
        s = sin(dtheta) / dtheta;
        c = (1 - cos(dtheta)) / dtheta;
    }
    double dx = s * local_change.x - c * local_change.y;
    double dy = c * local_change.x + s * local_change.y;
    current_pose.x += cos(current_pose.theta) * dx - sin(current_pose.theta) * dy;
    current_pose.y += sin(current_pose.theta) * dx + cos(current_pose.theta) * dy;
    current_pose.theta += dtheta;
}
}