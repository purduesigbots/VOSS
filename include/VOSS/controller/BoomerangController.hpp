#pragma once

#include "AbstractController.hpp"
#include "PIDController.hpp"
#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"

namespace voss::controller {

class BoomerangController : public AbstractController {
  protected:
    std::shared_ptr<BoomerangController> p;
    std::shared_ptr<PIDController> child = nullptr;
    double lead_pct;

  public:
    BoomerangController(std::shared_ptr<localizer::AbstractLocalizer> l);

    chassis::ChassisCommand get_command(bool reverse, bool thru) override;
    chassis::ChassisCommand get_angular_command(bool reverse,
                                                bool thru) override;

    void reset() override;

    std::shared_ptr<BoomerangController>
    modify_linear_constants(double kP, double kI, double kD);
    std::shared_ptr<BoomerangController>
    modify_angular_constants(double kP, double kI, double kD);
    std::shared_ptr<BoomerangController> modify_tracking_kp(double kP);
    std::shared_ptr<BoomerangController> modify_exit_error(double error);
    std::shared_ptr<BoomerangController>
    modify_angular_exit_error(double error);
    std::shared_ptr<BoomerangController> modify_min_error(double error);
    std::shared_ptr<BoomerangController> modify_lead_pct(double lead_pct);
    std::shared_ptr<BoomerangController> modify_settle_time(double time);

    friend class BoomerangControllerBuilder;
};

} // namespace voss::controller