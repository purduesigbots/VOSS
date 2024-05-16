/**
 * @file ADITrackingWheel.hpp
 * @brief
 * @version 0.1.2
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "pros/adi.hpp"
#include "VOSS/localizer/AbstractTrackingWheel.hpp"
#include <memory>

namespace voss::localizer {

class ADITrackingWheel : public AbstractTrackingWheel {
  private:
    std::unique_ptr<pros::adi::Encoder> encoder;

  protected:
    double get_raw_position() override;

  public:
    ADITrackingWheel(int adi_port);
    ADITrackingWheel(int smart_port, int adi_port);
    void reset() override;
};

} // namespace voss::localizer
