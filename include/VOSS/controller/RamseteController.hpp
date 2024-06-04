#pragma once

#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/trajectory/Trajectory.hpp"
#include "VOSS/utils/FeedFwd.hpp"
#include <memory>

namespace voss::controller {
class RamseteController : private std::enable_shared_from_this<RamseteController>,
                          public virtual IsTrajectoryFollowController {
  protected:
    std::shared_ptr<RamseteController> p;
    double zeta;
    double b;
    utils::FeedForward motor_ff;
    double track_width;
    long init_time;
  public:
    struct RamseteController_Construct_Params {
        double zeta = 0.7;
        double b = 2.0;
        double kV = 0.0;
        double kA = 0.0;
        double kS = 0.0;
        double kD = 0.0;
        double track_width = 10.0;
    };
    RamseteController(RamseteController_Construct_Params params);

    chassis::DiffChassisCommand
    get_command(std::shared_ptr<localizer::AbstractLocalizer> l, bool reverse,
                bool thru, std::shared_ptr<AbstractExitCondition> ec) override;

    chassis::DiffChassisCommand
    get_angular_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                        bool reverse, bool thru,
                        voss::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) override;
    
    std::shared_ptr<RamseteController> get_ptr();

    void reset() override;

    std::shared_ptr<RamseteController> modify_constants(double zeta, double b);
};
}