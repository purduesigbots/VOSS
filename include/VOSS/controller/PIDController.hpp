#pragma once

#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/utils/PID.hpp"
#include <memory>

namespace voss::controller {

class PIDController : public AbstractController {
  protected:
    std::shared_ptr<PIDController> p;

    utils::PID linear_pid, angular_pid;
    double min_error;
    bool can_reverse;

    double min_vel;
    bool turn_overshoot;

  public:
    struct Params {
        double lin_kp = 20;
        double lin_ki = 0;
        double lin_kd = 0;
        double ang_kp = 250;
        double ang_ki = 0;
        double ang_kd = 0;
        double min_error = 5;
        double min_vel = 100;
    };

    explicit PIDController(Params params);

    chassis::DiffChassisCommand
    get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                std::shared_ptr<AbstractExitCondition> ec,
                const velocity_pair& v_pair, bool reverse, bool thru) override;

    chassis::DiffChassisCommand
    get_angular_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                        std::shared_ptr<AbstractExitCondition> ec,
                        const velocity_pair& v_pair, bool reverse, bool thru,
                        voss::AngularDirection direction) override;

    void reset() override;

    std::shared_ptr<PIDController> modify_linear_constants(double kP, double kI,
                                                           double kD);
    std::shared_ptr<PIDController>
    modify_angular_constants(double kP, double kI, double kD);
    std::shared_ptr<PIDController> modify_min_error(double min_error);
};

} // namespace voss::controller