/**
 * @file SwingControllerBuilder.hpp
 * @brief
 * @version 0.1.2
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "VOSS/controller/SwingController.hpp"
#include "VOSS/controller/SwingControllerBuilder.hpp"
#include "VOSS/localizer/AbstractLocalizer.hpp"

namespace voss::controller {

class SwingControllerBuilder {
  private:
    SwingController ctrl;

  public:
    SwingControllerBuilder(std::shared_ptr<localizer::AbstractLocalizer> l);

    static SwingControllerBuilder
    new_builder(std::shared_ptr<localizer::AbstractLocalizer> l);

    static SwingControllerBuilder from(SwingController swc);

    SwingControllerBuilder& with_angular_constants(double kP, double kI,
                                                   double kD);

    std::shared_ptr<SwingController> build();
};

} // namespace voss::controller
