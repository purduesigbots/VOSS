#pragma once

#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"

namespace voss::controller {

class AbstractController {

  protected:
    std::shared_ptr<localizer::AbstractLocalizer> l;
    Pose target;
    double angular_target;

  public:
    AbstractController(std::shared_ptr<localizer::AbstractLocalizer> l);

    virtual chassis::ChassisCommand get_command(bool reverse, bool thru) = 0;
    virtual chassis::ChassisCommand get_angular_command(bool reverse,
                                                        bool thru) = 0;

    virtual void reset() = 0;

    void set_target(Pose target, bool relative);
    void set_angular_target(double angle, bool relative);
};

} // namespace voss::controller