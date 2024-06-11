#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>
#include <memory>
#include "iostream"

namespace voss::controller {

// Set desired postion with x, y, and heading
// Relative target position WIP
void AbstractController::set_target(Pose target) {
    printf("im here9\n");
    std::cout << this->target.x << '\n';
    this->target = target;
    printf("im here10\n");
}

// Set desired orientation
void AbstractController::set_angular_target(double angular_target) {
    this->angular_target = angular_target;
}

void AbstractController::set_target_path(const std::vector<Pose>& path) {
    this->target_path = path;
}

void AbstractController::set_target_trajectory(
    const trajectory::Trajectory& traj) {
    this->target_trajectory = std::make_shared<trajectory::Trajectory>(traj);
}

void AbstractController::set_target_trajectory(
    const trajectory::PreGenTrajectory& traj) {
    this->target_trajectory = std::make_shared<trajectory::PreGenTrajectory>(traj);
}

} // namespace voss::controller