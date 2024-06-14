#include "../include/VOSS/asset/Decode.hpp"
#include "VOSS/trajectory/PreGenTrajectory.hpp"
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <string_view>

namespace voss {
trajectory::PreGenTrajectory decode_csv(const asset& file) {
    std::map<double, trajectory::TrajectoryPose> t_to_trajectory;
//    std::string input(reinterpret_cast<char*>(file.buf), file.size);

    // TODO: implement path file protocol

    return {t_to_trajectory};
}
}; // namespace voss