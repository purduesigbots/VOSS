/**
 * @file IMETrackingWheel.hpp
 * @brief
 * @version 0.1.2
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "pros/motors.hpp"
#include "VOSS/localizer/AbstractTrackingWheel.hpp"
#include <memory>

namespace voss::localizer {

class IMETrackingWheel : public AbstractTrackingWheel {
  private:
    std::unique_ptr<pros::v5::Motor> encoder;

  protected:
    double get_raw_position() override;

  public:
    IMETrackingWheel(int port);
    void reset() override;
};

} // namespace voss::localizer
