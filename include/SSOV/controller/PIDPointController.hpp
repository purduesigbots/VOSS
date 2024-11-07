#pragma once

#include "SSOV/controller/PIDContoller.hpp"
#include "SSOV/controller/PointController.hpp"

namespace ssov {
    class PIDPointController: public PointController {
        private:
            PIDController linear_pid;
            PIDController angular_pid;
            double min_error;
            bool can_reverse;
        public:
            PIDPointController(PIDConstants linear_constants,
                               PIDConstants angular_constants,
                               double min_error):
                linear_pid(linear_constants),
                angular_pid(angular_constants),
                min_error(min_error) {};
            ChassisSpeeds compute(const Pose &current_pose, const Point &target_point, bool reverse, bool thru) override;
            void reset() override;
    };
}