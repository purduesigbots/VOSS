#pragma once

#include "voss/controller/PIDController.hpp"
#include "voss/localizer/AbstractLocalizer.hpp"
#include "voss/utils/flags.hpp"
#include "voss/utils/Point.hpp"

namespace voss::controller {
class PPController : public voss::controller::AbstractController {
  public:
    PPController(std::shared_ptr<voss::localizer::AbstractLocalizer> l);

    bool isArrived() const;
    chassis::DiffChassisCommand get_command(bool reverse, bool thru) override;
    chassis::DiffChassisCommand
    get_angular_command(bool reverse, bool thru,
                        voss::AngularDirection direction) override;

    void reset() override;

  protected:
    double linear_kP, linear_kI, linear_kD;
    double angular_kP, angular_kI, angular_kD;
    double tracking_kP;
    double exit_error;
    double angular_exit_error;
    double min_error;
    double can_reverse;
    double settle_time;

    double close;
    double close_2;
    int counter;
    double prev_angle;
    bool turn_overshoot;
    double min_speed;

    double prev_lin_err, total_lin_err, prev_ang_err, total_ang_err;

  private:
    bool arrived = true;
    double lookAheadDist;
    double slewStep;
    double prevOverallLinearError;
    double prevToNextPointError;
    double prevIndex;
    int closest();
    static voss::Point absToLocal(voss::Pose currentPose, voss::Point pt);
    voss::Point getLookAheadPoint(double lookAheadDist);
    double linear_pid(double error);
    double angular_pid(double error);

    template <typename T> static int sgn(T val) {
        return val >= 0 ? 1 : -1;
    }

    friend class PPControllerBuilder;
};
} // namespace voss::controller