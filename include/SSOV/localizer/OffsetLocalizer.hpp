#pragma once

#include "SSOV/localizer/Localizer.hpp"
#include "SSOV/common/Math.hpp"

namespace ssov {

class OffsetLocalizer {
    private:
        Localizer *localizer;
        Pose offset;
    public:
        OffsetLocalizer(Localizer *localizer, Pose offset): localizer(localizer), offset(offset) {};
        Pose get_pose() {
            Pose pose = localizer->get_pose();
            pose.theta = norm(pose.theta - offset.theta);
            pose.x -= cos(pose.theta) * offset.x - sin(pose.theta) * offset.y;
            pose.y -= sin(pose.theta) * offset.x + cos(pose.theta) * offset.y;
            return pose;
        }
        void set_pose(Pose pose) {
            pose.x += cos(pose.theta) * offset.x - sin(pose.theta) * offset.y;
            pose.y += sin(pose.theta) * offset.x + cos(pose.theta) * offset.y;
            pose.theta = norm(pose.theta + offset.theta);
            localizer->set_pose(pose);
        }
        Pose get_velocities() {
            Pose velocities = localizer->get_velocities();
            double rotated_x_vel = cos(-offset.theta) * velocities.x - sin(-offset.theta) * velocities.y;
            double rotated_y_vel = sin(-offset.theta) * velocities.x + cos(-offset.theta) * velocities.y;
            rotated_x_vel += velocities.theta * offset.y;
            rotated_y_vel -= velocities.theta * offset.x;
            return {rotated_x_vel, rotated_y_vel, velocities.theta};
        }
};

}