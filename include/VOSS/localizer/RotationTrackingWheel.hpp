/**
 * @file RotationTrackingWheel.hpp
 * @brief
 * @version 0.1.2
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "pros/rotation.hpp"
#include "VOSS/localizer/AbstractTrackingWheel.hpp"
#include <memory>

namespace voss::localizer {

class RotationTrackingWheel : public AbstractTrackingWheel {
  private:
    std::unique_ptr<pros::v5::Rotation> encoder;

  protected:
    double get_raw_position() override;

  public:
    RotationTrackingWheel(int port);
    void reset() override;
};

} // namespace voss::localizer
