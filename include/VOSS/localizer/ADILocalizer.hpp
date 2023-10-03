#pragma once

#include "pros/adi.hpp"
#include "voss/localizer/AbstractLocalizer.hpp"
#include <atomic>
#include <memory>

namespace voss::localizer {

class ADILocalizer : public AbstractLocalizer {

private:
  double prev_left_pos = {0.0}, prev_right_pos = {0.0}, prev_middle_pos = {0.0};
  Pose prev_pose = {0.0, 0.0, 0.0};

  double left_right_tpi = {0.0}, middle_tpi = {0.0};
  double track_width = {0.0};
  double left_right_dist = {0.0}, middle_dist = {0.0};

  std::unique_ptr<pros::adi::Encoder> left_encoder;
  std::unique_ptr<pros::adi::Encoder> right_encoder;
  std::unique_ptr<pros::adi::Encoder> middle_encoder;

public:
  ADILocalizer(int left, int right, int mid, double lr_tpi, double mid_tpi,
               double track_width, double middle_dist);

  int getLeftEncoderValue();
  int getRightEncoderValue();
  int getMiddleEncoderValue();

  void update();
};

} // namespace voss::localizer