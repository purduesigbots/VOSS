#pragma once

#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/utils/PID.hpp"
#include <memory>

namespace voss::controller {

class ArcPIDController : private std::enable_shared_from_this<ArcPIDController>,
                         public virtual IsMoveController {
  private:
  protected:
    std::shared_ptr<ArcPIDController> p;
    utils::PID linear_pid;
    utils::PID angular_pid;
    double track_width;
    double min_error;
    double can_reverse;
    double arc_radius;
    Point arc_center;
    double prev_t;

  public:
    struct Arc_Construct_Params {
        double lin_kp = 20;
        double lin_ki = 0;
        double lin_kd = 0;
        double ang_kp = 250;
        double ang_ki = 0;
        double ang_kd = 0;
        double track_width = 15;
        double min_error = 5;
    };

    ArcPIDController(Arc_Construct_Params params);

    chassis::DiffChassisCommand
    get_command(std::shared_ptr<localizer::AbstractLocalizer> l, bool reverse, bool thru,
                std::shared_ptr<AbstractExitCondition> ec) override;
    chassis::DiffChassisCommand
    get_angular_command(std::shared_ptr<localizer::AbstractLocalizer> l, bool reverse, bool thru,
                        voss::AngularDirection direction,
                        std::shared_ptr<AbstractExitCondition> ec) override;

    std::shared_ptr<ArcPIDController> get_ptr();

    void reset() override;

    std::shared_ptr<ArcPIDController>
    modify_linear_constants(double kP, double kI, double kD);
    std::shared_ptr<ArcPIDController> modify_track_width(double track_width);
    std::shared_ptr<ArcPIDController> modify_min_error(double error);
};

} // namespace voss::controller