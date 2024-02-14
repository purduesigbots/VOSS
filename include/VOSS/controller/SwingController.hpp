#pragma once

#include "PIDController.hpp"
#include "VOSS/controller/AbstractController.hpp"
#include <memory>

namespace voss::controller {
class SwingController : public AbstractController {
  protected:
    std::shared_ptr<SwingController> p;
    double angular_kP, angular_kI, angular_kD;
    double angular_exit_error;
    double settle_time;
    bool can_reverse;

    double close;
    double close_2;
    int counter;

    double prev_ang_err, total_ang_err;
    double prev_ang_speed;

  public:
    SwingController(std::shared_ptr<localizer::AbstractLocalizer> l);

    chassis::ChassisCommand get_command(bool reverse, bool thru) override;
    chassis::ChassisCommand get_angular_command(bool reverse,
                                                bool thru) override;

    double angular_pid(double error);

    void reset() override;

    std::shared_ptr<SwingController>
    modify_angular_constants(double kP, double kI, double kD);
    std::shared_ptr<SwingController> modify_angular_exit_error(double error);
    std::shared_ptr<SwingController> modify_settle_time(double time);

    friend class SwingControllerBuilder;
};
} // namespace voss::controller
