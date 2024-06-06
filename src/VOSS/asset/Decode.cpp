#include "../include/VOSS/asset/Decode.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <map>
#include "VOSS/trajectory/PreGenTrajectory.hpp"

namespace voss::asset {
trajectory::PreGenTrajectory decode_csv(const asset& file) {
    std::map<double, trajectory::TrajectoryPose> t_to_trajectory;
    std::string input(reinterpret_cast<char*>(file.buf), file.size);

//TODO: implement path file protocol


    return {t_to_trajectory};
}

}; // namespace voss::asset