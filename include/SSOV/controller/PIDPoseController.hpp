#pragma once

#include <memory>

#include "SSOV/controller/PIDContoller.hpp"
#include "SSOV/controller/PoseController.hpp"

namespace ssov {
    class PIDPoseController: public PoseController {
        private:
            PIDController linear_pid;
            PIDController angular_pid;
            const double min_error;
            bool can_reverse;
            // when within min_error from the target, min_dist_angle
            // is set to the current heading, and becomes the target
            // angle for the angular pid
            double min_dist_angle;
            bool debug = false;
        public:
            PIDPoseController(PIDConstants linear_constants,
                               PIDConstants angular_constants,
                               double min_error):
                linear_pid(linear_constants),
                angular_pid(angular_constants),
                min_error(min_error) {};
            static std::shared_ptr<PIDPoseController> create(PIDConstants linear_constants,
                                                              PIDConstants angular_constants,
                                                              double min_error) {
                return std::make_shared<PIDPoseController>(linear_constants, angular_constants, min_error);
            }
            void set_debug(bool debug) {
                this->debug = debug;
            }
            std::shared_ptr<PIDPoseController> modify_linear_pid(PIDConstants linear_constants) {
                auto mod = PIDPoseController::create(linear_constants, angular_pid.get_constants(), min_error);
                mod->set_debug(debug);
                return mod;
            }
            std::shared_ptr<PIDPoseController> modify_angular_pid(PIDConstants angular_constants) {
                auto mod = PIDPoseController::create(linear_pid.get_constants(), angular_constants, min_error);
                mod->set_debug(debug);
                return mod;
            }
            std::shared_ptr<PIDPoseController> modify_min_error(double min_error) {
                auto mod = PIDPoseController::create(linear_pid.get_constants(), angular_pid.get_constants(), min_error);
                mod->set_debug(debug);
                return mod;
            }

            DriveSignal compute(const Pose &current_pose, const Pose &target_point, bool reverse, bool thru, bool holonomic, int strafe_angle=0) override;
            void reset() override;
    };
}