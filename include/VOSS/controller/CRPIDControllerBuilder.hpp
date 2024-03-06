#pragma once

#include "VOSS/controller/CRPIDController.hpp"

namespace voss::controller {
class CRPIDControllerBuilder {
private:
    CRPIDController ctrl;
public:
    CRPIDControllerBuilder(std::shared_ptr<localizer::AbstractLocalizer> l);
    static CRPIDControllerBuilder new_builder(std::shared_ptr<localizer::AbstractLocalizer> l);

    CRPIDControllerBuilder& with_linear_constants(double kP, double kI, double kD);
    CRPIDControllerBuilder& with_angular_constants(double kP, double kI,
                                                 double kD);
    CRPIDControllerBuilder& with_r(double r);
    CRPIDControllerBuilder& with_exit_error(double error);
    CRPIDControllerBuilder& with_angular_exit_error(double error);
    CRPIDControllerBuilder& with_min_error(double error);
    CRPIDControllerBuilder& with_settle_time(double time);

    std::shared_ptr<CRPIDController> build();
};
}