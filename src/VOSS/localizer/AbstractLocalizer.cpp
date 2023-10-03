#include "voss/localizer/AbstractLocalizer.hpp"
#include <cmath>

namespace voss::localizer {

AbstractLocalizer::AbstractLocalizer() {
  this->mtx = new pros::Mutex;
  this->pose = {0.0, 0.0, 0.0};
}

void AbstractLocalizer::begin_localization() {
  pros::Task localization_task([=]() {
    while (true) {
      if (this->mtx->try_lock()) {
        this->update();
        this->mtx->unlock();
      }
      pros::delay(10);
    }
  });
}

void AbstractLocalizer::set_pose(Pose pose) {
  while (!this->mtx->try_lock())
    pros::delay(10);

  this->pose = pose;
  this->mtx->unlock();
}

Pose AbstractLocalizer::get_pose() {
  while (!this->mtx->try_lock())
    pros::delay(10);

  Pose ret = this->pose;
  this->mtx->unlock();
  return ret;
}

double AbstractLocalizer::get_orientation_rad() {
  while (!this->mtx->try_lock())
    pros::delay(10);

  double ret = this->pose.theta;
  this->mtx->unlock();
  return ret;
}

double AbstractLocalizer::get_orientation_deg() {
  while (!this->mtx->try_lock())
    pros::delay(10);

  double ret = this->pose.theta * 180 * M_1_PI;
  this->mtx->unlock();
  return ret;
}

Point AbstractLocalizer::get_position() {
  while (!this->mtx->try_lock())
    pros::delay(10);

  Point ret{this->pose.x, this->pose.y};
  this->mtx->unlock();
  return ret;
}
} // namespace voss::localizer