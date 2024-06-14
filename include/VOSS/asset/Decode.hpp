#pragma once
#include "../utils/Pose.hpp"
#include "VOSS/asset/Asset.hpp"
#include "VOSS/trajectory/PreGenTrajectory.hpp"

namespace voss {
voss::trajectory::PreGenTrajectory decode_csv(const asset& file);
};


