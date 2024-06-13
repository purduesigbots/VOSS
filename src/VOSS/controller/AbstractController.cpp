#include "VOSS/controller/AbstractController.hpp"
#include "iostream"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/utils/angle.hpp"
#include <cmath>
#include <memory>

namespace voss::controller {

chassis::DiffChassisCommand
AbstractController::get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                                std::shared_ptr<AbstractExitCondition> ec,
                                const AbstractController::velocity_pair& v_pair,
                                bool reverse, bool thru) {
    return chassis::DiffChassisCommand{chassis::Stop{}};
}

chassis::DiffChassisCommand AbstractController::get_angular_command(
    std::shared_ptr<localizer::AbstractLocalizer> l,
    std::shared_ptr<AbstractExitCondition> ec,
    const AbstractController::velocity_pair& v_pair, bool reverse, bool thru,
    voss::AngularDirection direction) {
    return chassis::DiffChassisCommand{chassis::Stop{}};
}

// Set desired postion with x, y, and heading
// Relative target position WIP
void AbstractController::set_target_pose(const Pose& target_pose) {
    this->target_pose = target_pose;
}

// Set desired orientation
void AbstractController::set_target_angle(double target_angle) {
    this->target_angle = target_angle;
}

void AbstractController::set_target_path(const std::vector<Pose>& path) {
    this->target_path = path;
}

void AbstractController::set_target_trajectory(
    const trajectory::Trajectory& target_trajectory) {
    this->target_trajectory =
        std::make_shared<trajectory::Trajectory>(target_trajectory);
}

void AbstractController::set_target_trajectory(
    const trajectory::PreGenTrajectory& target_trajectory) {
    this->target_trajectory =
        std::make_shared<trajectory::PreGenTrajectory>(target_trajectory);
}

} // namespace voss::controller