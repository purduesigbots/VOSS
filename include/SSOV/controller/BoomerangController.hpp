#pragma once

#include <memory>

#include "SSOV/controller/PoseController.hpp"
#include "SSOV/controller/PIDContoller.hpp"

namespace ssov {
    class BoomerangController: public PoseController {
        private:
            PIDController linear_pid;
            PIDController angular_pid;
            const double min_error;
            const double lead_pct;
            bool can_reverse;
            bool debug = false;
        public:
            BoomerangController(PIDConstants linear_constants,
                                PIDConstants angular_constants,
                                double min_error,
                                double lead_pct):
                linear_pid(linear_constants),
                angular_pid(angular_constants),
                min_error(min_error),
                lead_pct(lead_pct) {};
            static std::shared_ptr<BoomerangController> create(PIDConstants linear_constants,
                                                               PIDConstants angular_constants,
                                                               double min_error,
                                                               double lead_pct) {
                return std::make_shared<BoomerangController>(linear_constants, angular_constants, min_error, lead_pct);
            }

            DriveSignal compute(const Pose &current_pose, const Pose &target_pose, bool reverse, bool thru) override;
            void reset() override;
            void set_debug(bool debug) {
                this->debug = debug;
            }
            std::shared_ptr<BoomerangController> modify_angular_pid(PIDConstants angular_constants) {
                auto mod = BoomerangController::create(linear_pid.get_constants(), angular_constants, min_error, lead_pct);
                mod->set_debug(debug);
                return mod;
            }
            std::shared_ptr<BoomerangController> modify_lead_pct(double lead_pct) {
                auto mod = BoomerangController::create(linear_pid.get_constants(), angular_pid.get_constants(), min_error, lead_pct);
                mod->set_debug(debug);
                return mod;
            }
    };
}