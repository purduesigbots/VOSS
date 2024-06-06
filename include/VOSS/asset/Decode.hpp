#pragma once
#include "../utils/Pose.hpp"
#include "Asset.hpp"
#include "VOSS/trajectory/PreGenTrajectory.hpp"

namespace voss::asset {
trajectory::PreGenTrajectory decode_csv(const asset& file);
}; // namespace voss
