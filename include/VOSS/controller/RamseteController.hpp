#pragma once

#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/trajectory/Trajectory.hpp"
#include "VOSS/utils/FeedFwd.hpp"
#include "VOSS/utils/PID.hpp"
#include <memory>

namespace voss::controller {
class RamseteController : public AbstractController {

  public:
    struct Params {
        double zeta = 0.7;
        double b = 2.0;
        double ffwd_kV = 10.0;
        double ffwd_kS = 10.0;
        double ffwd_kA = 10.0;
        double ffwd_kD = 10.0;
        double track_width = 12.5;
        double wheel_diameter = 3.25;
        double gear_ratio;
    };
    explicit RamseteController(Params params);


    struct PID_Params {
        double left_kP = 0.0;
        double left_kI = 0.0;
        double left_kD = 0.0;
        double right_kP = 0.0;
        double right_kI = 0.0;
        double right_kD = 0.0;
    };
    std::shared_ptr<RamseteController> with_velocity_controller(PID_Params controller);

  public:
    chassis::DiffChassisCommand
    get_command(std::shared_ptr<localizer::AbstractLocalizer> l,
                std::shared_ptr<AbstractExitCondition> ec,
                const velocity_pair& v_pair, bool reverse, bool thru) override;

    static std::shared_ptr<RamseteController> create_controller(Params params);

    void reset() override;

    std::shared_ptr<RamseteController> modify_constants(double zeta, double b);

  protected:
    std::shared_ptr<RamseteController> p;
    double zeta;
    double b;
    utils::FeedForward motor_ff;
    utils::PID left_motor_pid;
    utils::PID right_motor_pid;
    double track_width;
    long init_time;
    bool has_vel_pid_controller;
    double gear_ratio;
    double wheel_diameter;
};
} // namespace voss::controller