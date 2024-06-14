#pragma once

#include "PIDController.hpp"
#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/utils/PID.hpp"
#include <memory>

namespace voss::controller {
class SwingController : public AbstractController {

  public:
    struct Params {
        double ang_kp = 250;
        double ang_ki = 0;
        double ang_kd = 0;
        double min_error = 5;
    };

  public:
    explicit SwingController(Params params);

    static std::shared_ptr<SwingController> create_controller(Params params);

    chassis::DiffChassisCommand
    get_angular_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                        std::shared_ptr<AbstractExitCondition> ec,
                        const velocity_pair& v_pair, bool reverse, bool thru,
                        voss::AngularDirection direction) override;

    void reset() override;

    std::shared_ptr<SwingController>
    modify_angular_constants(double kP, double kI, double kD);

  protected:
    std::shared_ptr<SwingController> p;
    utils::PID angular_pid;
    bool can_reverse;
    bool turn_overshoot;

    double prev_ang_speed;
};
} // namespace voss::controller
