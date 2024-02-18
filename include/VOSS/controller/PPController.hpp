#pragma once

#include "voss/utils/Point.hpp"
#include "voss/controller/PIDController.hpp"
#include "voss/utils/flags.hpp"

namespace voss::controller{
class PPController: public voss::controller::AbstractController {
public:
    PPController(std::shared_ptr<voss::localizer::AbstractLocalizer> l);

    bool isArrived() const;

private:
  chassis::DiffChassisCommand get_command(bool reverse, bool thru) override;
  chassis::DiffChassisCommand get_angular_command(bool reverse, bool thru, voss::AngularDirection direction) override;

  void reset() override;

    int closest(const std::vector<voss::Point>& path);
    voss::Point absToLocal(voss::Point pt);
    voss::Point getLookAheadPoint(const std::vector<voss::Point>& path, double lookAheadDist);
    // std::pair<double, double> step(const std::vector<voss::Point>& path, bool reverse, bool thru, bool relative, double lookAheadDist, double maxVelocity);
    double slew(double target, bool is_left);

    template <typename T> static int sgn(T val) {
        return val >= 0 ? 1 : -1;
    }

    std::shared_ptr<voss::controller::PIDController> pidController;
    std::shared_ptr<voss::localizer::AbstractLocalizer> localizer;

    bool arrived = true;
    double lookAheadDist;
    double slewStep;
    double prevOverallLinearError;
    double prevToNextPointError;
    double prevIndex;
    std::pair<double, double> prevVoltages = {0, 0};
    std::unique_ptr<pros::MotorGroup> leftMotors;
    std::unique_ptr<pros::MotorGroup> rightMotors;

    friend class PPControllerBuilder;
};
} // namespace voss::controller