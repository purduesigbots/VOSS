#include "legs/motion_controllers/boomerang_controller.hpp"

#include "api.h"

namespace legs {

class BoomerangController {
    private:
        double linear_exit_error;
        double angular_exit_error;

        // settling
        double settle_thresh_linear;
        double settle_thresh_angular;
        int settle_time;
}

void BoomerangController::move(Eigen::Vector3d& target) {

}

void BoomerangController::turn(double target) {

}

}