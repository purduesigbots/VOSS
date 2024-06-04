#pragma once

#include "PIDController.hpp"
#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/utils/PID.hpp"
#include <memory>

namespace voss::controller {
class SwingController : private std::enable_shared_from_this<SwingController>,
                        public virtual IsTurnController {
  protected:
    std::shared_ptr<SwingController> p;
    utils::PID angular_pid;
    bool can_reverse;
    bool turn_overshoot;

    double prev_ang_speed;

  public:
    struct SwingController_Construct_Params {
        double ang_kp = 250;
        double ang_ki = 0;
        double ang_kd = 0;
        double min_error = 5;
    };

    SwingController(SwingController_Construct_Params params);

    chassis::DiffChassisCommand
    get_command(std::shared_ptr<localizer::AbstractLocalizer> l, bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) override;
    chassis::DiffChassisCommand
    get_angular_command(std::shared_ptr<localizer::AbstractLocalizer> l, bool reverse, bool thru,
                        voss::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) override;

    std::shared_ptr<SwingController> get_ptr();

    void reset() override;

    std::shared_ptr<SwingController>
    modify_angular_constants(double kP, double kI, double kD);

    friend class SwingControllerBuilder;
};
} // namespace voss::controller
