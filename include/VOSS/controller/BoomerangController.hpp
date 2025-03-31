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
    enum class ThruBehavior { // affects calculation of lin_speed on thru movements
      MIN_VEL, // use linear PID, but enforce a minimum velocity
      FULL_SPEED // no PID, go full speed
    };
    enum class CosineScaling {
      MIN_ERR, // only do cosine scaling within the minimum err
      ALL_THE_TIME // do cosine scaling all the time
    };
    enum class MinErrBehavior {
      TARGET_HEADING, // directly switch from turning towards carrot point to turning towards target heading
      SCALE_TARGET_HEADING, // scale between turning towards carrot point to turning towards target point
    };
  protected:
    std::shared_ptr<BoomerangController> p;
    double lead_pct;

    utils::PID linear_pid, angular_pid;
    double min_error;
    bool can_reverse;

    double min_vel;

    ThruBehavior thru_behavior = ThruBehavior::FULL_SPEED;
    CosineScaling cosine_scaling = CosineScaling::ALL_THE_TIME;
    MinErrBehavior min_err_behavior = MinErrBehavior::SCALE_TARGET_HEADING;
    bool enable_overturn = false;

  public:
    BoomerangController(std::shared_ptr<localizer::AbstractLocalizer> l);

    chassis::DiffChassisCommand
    get_command(bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) override;
    chassis::DiffChassisCommand
    get_angular_command(bool reverse, bool thru,
                        voss::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) override;

    void reset() override;

    std::shared_ptr<BoomerangController>
    modify_linear_constants(double kP, double kI, double kD);
    std::shared_ptr<BoomerangController>
    modify_angular_constants(double kP, double kI, double kD);
    std::shared_ptr<BoomerangController> modify_min_error(double error);
    std::shared_ptr<BoomerangController> modify_lead_pct(double lead_pct);

    void set_thru_behavior(ThruBehavior thru_behavior);
    void set_cosine_scaling(CosineScaling cosine_scaling);
    void set_min_err_behavior(MinErrBehavior min_err_behavior);
    void set_overturn(bool overturn);

    friend class BoomerangControllerBuilder;
};

} // namespace voss::controller