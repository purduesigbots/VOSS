#pragma once

#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/trajectory/Trajectory.hpp"
#include "VOSS/utils/FeedFwd.hpp"
#include <memory>

namespace voss::controller {
class RamseteController : public AbstractController {
  protected:
    std::shared_ptr<RamseteController> p;
    double zeta;
    double b;
    utils::FeedForward motor_ff;
    double track_width;
    long init_time;

  public:
    struct Params {
        double zeta = 0.7;
        double b = 2.0;
        double kV = 0.0;
        double kA = 0.0;
        double kS = 0.0;
        double kD = 0.0;
        double track_width = 10.0;
    };
    explicit RamseteController(Params params);

    chassis::DiffChassisCommand
    get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                std::shared_ptr<AbstractExitCondition> ec,
                const velocity_pair& v_pair, bool reverse, bool thru) override;

    void reset() override;

    std::shared_ptr<RamseteController> modify_constants(double zeta, double b);
};
} // namespace voss::controller