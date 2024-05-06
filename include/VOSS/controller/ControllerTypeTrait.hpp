#pragma once
#include "ArcPIDController.hpp"
#include "BoomerangController.hpp"
#include "PIDController.hpp"
#include "SwingController.hpp"

namespace voss::controller {

template <typename T> struct Is_linear_motion : std::false_type {};
template <> struct Is_linear_motion<PIDController> : std::true_type {};
template <> struct Is_linear_motion<BoomerangController> : std::true_type {};
template <> struct Is_linear_motion<ArcPIDController> : std::true_type {};

template <typename T> struct Is_angular_motion : std::false_type {};
template <> struct Is_angular_motion<PIDController> : std::true_type {};
template <> struct Is_angular_motion<SwingController> : std::true_type {};

template <typename T> struct Is_path_follow_motion : std::false_type {};

}; // namespace voss::controller