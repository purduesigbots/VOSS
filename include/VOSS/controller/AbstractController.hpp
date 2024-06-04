#pragma once

#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/utils/flags.hpp"
#include "VOSS/trajectory/Trajectory.hpp"

namespace voss::controller {

class AbstractController {

  protected:
    Pose target;
    double angular_target;
    std::vector<Pose> target_path;
    std::optional<trajectory::Trajectory> target_trajectory;

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

    void set_target(const Pose& target);
    void set_angular_target(double angle);
    void set_target_path(const std::vector<Pose>& path);
    void set_target_trajectory(const trajectory::Trajectory& traj);
};

class IsMoveController : virtual public AbstractController {};
class IsTurnController : virtual public AbstractController {};
class IsPathFollowController : virtual public AbstractController {};
class IsTrajectoryFollowController : virtual public AbstractController {};


} // namespace voss::controller