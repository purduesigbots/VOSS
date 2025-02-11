#pragma once

#include <memory>

#include "SSOV/controller/PIDContoller.hpp"
#include "SSOV/controller/PointController.hpp"

namespace ssov {
    class PIDPointController: public PointController {
        private:
            PIDController linear_pid;
            PIDController angular_pid;
            const double min_error;
            bool can_reverse;
        public:
            PIDPointController(PIDConstants linear_constants,
                               PIDConstants angular_constants,
                               double min_error):
                linear_pid(linear_constants),
                angular_pid(angular_constants),
                min_error(min_error) {};
            static std::shared_ptr<PIDPointController> create(PIDConstants linear_constants,
                                                              PIDConstants angular_constants,
                                                              double min_error) {
                return std::make_shared<PIDPointController>(linear_constants, angular_constants, min_error);
            }
            std::shared_ptr<PIDPointController> modify_linear_pid(PIDConstants linear_constants) {
                return PIDPointController::create(linear_constants, angular_pid.get_constants(), min_error);
            }
            std::shared_ptr<PIDPointController> modify_angular_pid(PIDConstants angular_constants) {
                return PIDPointController::create(linear_pid.get_constants(), angular_constants, min_error);
            }
            std::shared_ptr<PIDPointController> modify_min_error(double min_error) {
                return PIDPointController::create(linear_pid.get_constants(), angular_pid.get_constants(), min_error);
            }

            DriveSignal compute(const Pose &current_pose, const Point &target_point, bool reverse, bool thru) override;
            void reset() override;
    };
}