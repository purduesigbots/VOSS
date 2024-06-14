#pragma once
#include "AbstractController.hpp"
#include "ArcPIDController.hpp"
#include "BoomerangController.hpp"
#include "PIDController.hpp"
#include "PPController.hpp"
#include "RamseteController.hpp"
#include "SwingController.hpp"

namespace voss::controller {

template <class ControllerType>
concept IsController =
    requires { std::is_base_of<ControllerType, AbstractController>(); };

template <typename T> struct is_move_motion : std::false_type {};
template <> struct is_move_motion<PIDController> : std::true_type {};
template <> struct is_move_motion<BoomerangController> : std::true_type {};
template <> struct is_move_motion<ArcPIDController> : std::true_type {};
template <typename T> constexpr inline bool is_move_motion_v() {
    return is_move_motion<T>::value;
};
template <class ControllerType>
concept MoveController = is_move_motion_v<ControllerType>();

template <typename T> struct is_angular_motion : std::false_type {};
template <> struct is_angular_motion<PIDController> : std::true_type {};
template <> struct is_angular_motion<SwingController> : std::true_type {};
template <typename T> constexpr inline bool is_angular_motion_v() {
    return is_angular_motion<T>::value;
};
template <class ControllerType>
concept AngularController = is_angular_motion_v<ControllerType>();

template <typename T> struct is_path_follow_motion : std::false_type {};
template <> struct is_path_follow_motion<PPController> : std::true_type {};
template <typename T> constexpr inline bool is_path_follow_motion_v() {
    return is_path_follow_motion<T>::value;
};
template <class ControllerType>
concept PathFollowController = is_path_follow_motion_v<ControllerType>();

template <typename T> struct is_trajectory_follow_motion : std::false_type {};
template <>
struct is_trajectory_follow_motion<RamseteController> : std::true_type {};
template <typename T> constexpr inline bool is_trajectory_follow_motion_v() {
    return is_trajectory_follow_motion<T>::value;
};
template <class ControllerType>
concept TrajectoryFollowController =
    is_trajectory_follow_motion_v<ControllerType>();

}; // namespace voss::controller