#include "SSOV/trajectory/Path.hpp"

namespace ssov {

PathSample Path::sample(double dist_resolution) const {
    double length = this->length();
    int num_samples = std::max(1.0, std::ceil(length / dist_resolution));

    PathSample result;
    result.poses.reserve(num_samples + 1);
    result.distances.reserve(num_samples + 1);
    for (int i = 0; i <= num_samples; i++) {
        double distance = length / num_samples * i;
        PoseWithCurvature pose = this->at(distance);
        result.distances.push_back(distance);
        result.poses.push_back(pose);
    }

    return result;
}

}