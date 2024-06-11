#pragma once

#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/utils/flags.hpp"
#include "VOSS/trajectory/Trajectory.hpp"
#include "VOSS/trajectory/PreGenTrajectory.hpp"

namespace voss::controller {

class AbstractController {
  protected:
    Pose target{0, 0, std::nullopt};
    double angular_target{0};
    std::vector<Pose> target_path{};
    std::shared_ptr<trajectory::AbstractTrajectory> target_trajectory{};

  public:

    friend class AbstractExitCondition;

    virtual chassis::DiffChassisCommand
    get_command(std::shared_ptr<localizer::AbstractLocalizer> l, bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) = 0;
    virtual chassis::DiffChassisCommand
    get_angular_command(std::shared_ptr<localizer::AbstractLocalizer> l, bool reverse, bool thru,
                        voss::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) = 0;

    virtual void reset() = 0;

    void set_target(Pose target);
    void set_angular_target(double angle);
    void set_target_path(const std::vector<Pose>& path);
    void set_target_trajectory(const trajectory::Trajectory& traj);
    void set_target_trajectory(const trajectory::PreGenTrajectory& gen);
};

class IsMoveController : virtual public AbstractController {
  public:
    IsMoveController() = default;
};
class IsTurnController : virtual public AbstractController {
  public:
    IsTurnController() = default;
};
class IsPathFollowController : virtual public AbstractController {
  public:
    IsPathFollowController() = default;
};
class IsTrajectoryFollowController : virtual public AbstractController {
  public:
    IsTrajectoryFollowController() = default;
};


} // namespace voss::controller