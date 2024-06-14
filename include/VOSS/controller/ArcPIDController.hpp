#pragma once

#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/utils/PID.hpp"
#include <memory>

namespace voss::controller {

class ArcPIDController : public AbstractController {
  public:
    struct Params {
        double lin_kp = 20;
        double lin_ki = 0;
        double lin_kd = 0;
        double ang_kp = 250;
        double ang_ki = 0;
        double ang_kd = 0;
        double track_width = 15;
        double min_error = 5;
    };

  public:
    explicit ArcPIDController(Params params);

    static std::shared_ptr<ArcPIDController> create_controller(Params params);

    chassis::DiffChassisCommand
    get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                std::shared_ptr<AbstractExitCondition> ec,
                const velocity_pair& v_pair, bool reverse, bool thru) override;

    void reset() override;

    std::shared_ptr<ArcPIDController>
    modify_linear_constants(double kP, double kI, double kD);
    std::shared_ptr<ArcPIDController> modify_track_width(double track_width);
    std::shared_ptr<ArcPIDController> modify_min_error(double error);

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
};

} // namespace voss::controller