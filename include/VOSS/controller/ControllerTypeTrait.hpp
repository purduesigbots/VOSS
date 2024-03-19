#pragma once
#include "AbstractController.hpp"
#include "ArcPIDController.hpp"
#include "BoomerangController.hpp"
#include "PIDController.hpp"
#include "SwingController.hpp"
#include <type_traits>


namespace voss::controller {

template <typename T> struct can_turn : std::false_type {};

template <>
struct can_turn<voss::controller::AbstractController> : std::true_type {};

template <>
struct can_turn<voss::controller::SwingController> : std::true_type {};

template <>
struct can_turn<voss::controller::PIDController> : std::true_type {};

template <typename T> struct can_move : std::false_type {};

template <>
struct can_move<voss::controller::AbstractController> : std::true_type {};

template <>
struct can_move<voss::controller::PIDController> : std::true_type {};

template <>
struct can_move<voss::controller::BoomerangController> : std::true_type {};

template <>
struct can_move<voss::controller::ArcPIDController> : std::true_type {};

}; // namespace voss::controller