#pragma once

#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/trajectory/PreGenTrajectory.hpp"
#include "VOSS/trajectory/Trajectory.hpp"
#include "VOSS/utils/flags.hpp"
#include <type_traits>

namespace voss::controller {

template <typename ControllerType>
    requires requires { typename ControllerType::Params; }
constexpr inline std::shared_ptr<ControllerType>
create_controller(typename ControllerType::Params params) {
    static_assert(
        std::is_constructible<ControllerType, typename ControllerType::Params>{},
        "No matching constructor");
    return std::make_shared<ControllerType>(params);
}

// get_command, get_angular_command
// store targets: target_pose, angular_target, target_trajectory
// (std::shared_ptr<AbstractTrajectory>), target_path
// able to differentiate different types of motion controller

class AbstractController {
  public:
    struct velocity_pair {
        double left, right;
    };

    virtual chassis::DiffChassisCommand
    get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                std::shared_ptr<AbstractExitCondition> ec,
                const velocity_pair& v_pair, bool reverse, bool thru);

    virtual chassis::DiffChassisCommand
    get_angular_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                        std::shared_ptr<AbstractExitCondition> ec,
                        const velocity_pair& v_pair, bool reverse, bool thru,
                        voss::AngularDirection direction);

    virtual void reset() = 0;
  public:
    void set_target_pose(const Pose& target_pose);
    void set_target_angle(double target_angle);
    void set_target_path(const std::vector<Pose>& target_path);
    void set_target_trajectory(
        const trajectory::Trajectory& target_trajectory);

    void set_target_trajectory(
        const trajectory::PreGenTrajectory& target_trajectory);

  protected:
    AbstractController() = default;

  protected:
    struct Params {};

    Pose target_pose{0, 0, std::nullopt};
    double target_angle{0};
    std::vector<Pose> target_path{};
    std::shared_ptr<trajectory::AbstractTrajectory> target_trajectory{nullptr};
};

} // namespace voss::controller
