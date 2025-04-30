#pragma once

#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/utils/PID.hpp"
#include <memory>

namespace voss::controller {

class PIDController : public AbstractController {
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
      NO_ANG_PID, // no angular PID, set ang_speed to 0
      SCALE_ANG_PID, // scale output of angular pid so it reaches 0 at half of min_err
      MAINTAIN_HEADING // maintain heading of when it entered min_err
    };
  protected:
    std::shared_ptr<PIDController> p;

    utils::PID linear_pid, angular_pid;
    double min_error;
    bool can_reverse;
    double min_err_heading;

    double min_vel;
    bool turn_overshoot;

    ThruBehavior thru_behavior = ThruBehavior::FULL_SPEED;
    CosineScaling cosine_scaling = CosineScaling::MIN_ERR;
    MinErrBehavior min_err_behavior = MinErrBehavior::MAINTAIN_HEADING;
    bool enable_overturn = false;

  public:
    PIDController(std::shared_ptr<localizer::AbstractLocalizer> l);

    chassis::DiffChassisCommand
    get_command(bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) override;
    chassis::DiffChassisCommand
    get_angular_command(bool reverse, bool thru,
                        voss::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) override;

    void reset() override;

    std::shared_ptr<PIDController> modify_linear_constants(double kP, double kI,
                                                           double kD);
    std::shared_ptr<PIDController>
    modify_angular_constants(double kP, double kI, double kD);
    std::shared_ptr<PIDController> modify_min_error(double min_error);

    void set_thru_behavior(ThruBehavior thru_behavior);
    void set_cosine_scaling(CosineScaling cosine_scaling);
    void set_min_err_behavior(MinErrBehavior min_err_behavior);
    void set_overturn(bool overturn);

    friend class PIDControllerBuilder;
    friend class BoomerangControllerBuilder;
};

} // namespace voss::controller