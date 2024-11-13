#pragma once

#include "SSOV/controller/TrajectoryFollower.hpp"

namespace ssov {

class RamseteTrajectoryFollower: public TrajectoryFollower {
    private:
        const double b;
        const double zeta;
        const double kV_lin;
        const double kV_ang;
        const double kP_lin;
        const double kP_ang;
    public:
        RamseteTrajectoryFollower(double b, double zeta, double kV_lin, double kV_ang, double kP_lin, double kP_ang)
            : b(b), zeta(zeta), kV_lin(kV_lin), kV_ang(kV_ang), kP_lin(kP_lin), kP_ang(kP_ang) {};
        ChassisSpeeds compute(Pose current_pose, Pose current_velocities, TrajectoryState target_state) override;
};

}