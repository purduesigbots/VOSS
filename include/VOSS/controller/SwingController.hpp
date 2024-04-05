#pragma once

#include "PIDController.hpp"
#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/utils/PID.hpp"
#include <memory>

namespace voss::controller {
class SwingController : public AbstractController {
  protected:
    std::shared_ptr<SwingController> p;
    utils::PID angular_pid;
    bool can_reverse;
    bool turn_overshoot;

    double prev_ang_speed;

  public:
    SwingController(std::shared_ptr<localizer::AbstractLocalizer> l);

    chassis::DiffChassisCommand
    get_command(bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) override;
    chassis::DiffChassisCommand
    get_angular_command(bool reverse, bool thru,
                        voss::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) override;

    void reset() override;

    std::shared_ptr<SwingController>
    modify_angular_constants(double kP, double kI, double kD);

    friend class SwingControllerBuilder;
};
} // namespace voss::controller
