#pragma once

#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/utils/flags.hpp"

namespace voss::controller {

class AbstractController {

  protected:
    std::shared_ptr<localizer::AbstractLocalizer> l;
    Pose target;
    double angular_target;
    std::vector<Point> target_path;

  public:
    AbstractController(std::shared_ptr<localizer::AbstractLocalizer> l);

    virtual chassis::DiffChassisCommand get_command(bool reverse,
                                                    bool thru) = 0;
    virtual chassis::DiffChassisCommand
    get_angular_command(bool reverse, bool thru,
                        voss::AngularDirection direction) = 0;

    virtual void reset() = 0;

    void set_target(Pose target, bool relative);
    void set_angular_target(double angle, bool relative);
    void set_target_path(const std::vector<Point> &path, bool relative); 
};

} // namespace voss::controller