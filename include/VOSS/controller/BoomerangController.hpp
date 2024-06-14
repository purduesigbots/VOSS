#pragma once

#include "AbstractController.hpp"
#include "PIDController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/exit_conditions/AbstractExitCondition.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/utils/flags.hpp"
#include "VOSS/utils/PID.hpp"

namespace voss::controller {

class BoomerangController : public AbstractController {
  public:
    struct Params {
        double lin_kp = 20;
        double lin_ki = 0;
        double lin_kd = 0;
        double ang_kp = 250;
        double ang_ki = 0;
        double ang_kd = 0;
        double lead_pct = 0.5;
        double min_error = 5;
        double min_vel = 100;
    };

  public:
    explicit BoomerangController(Params params);

    static std::shared_ptr<BoomerangController> create_controller(Params params);

    chassis::DiffChassisCommand
    get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                std::shared_ptr<AbstractExitCondition> ec,
                const velocity_pair& v_pair, bool reverse, bool thru) override;

    void reset() override;

    std::shared_ptr<BoomerangController>
    modify_linear_constants(double kP, double kI, double kD);
    std::shared_ptr<BoomerangController>
    modify_angular_constants(double kP, double kI, double kD);
    std::shared_ptr<BoomerangController> modify_min_error(double error);
    std::shared_ptr<BoomerangController> modify_lead_pct(double lead_pct);

  protected:
    std::shared_ptr<BoomerangController> p;
    double lead_pct;
    Pose carrotPoint;

    utils::PID linear_pid, angular_pid;
    double min_error;
    bool can_reverse;

    double min_vel;
};

} // namespace voss::controller