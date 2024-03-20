#pragma once

namespace voss::utils {

class AlphaBetaFilter {
private:
    double alpha;
    double beta;
    double position;
    double velocity;
public:
    AlphaBetaFilter(double alpha, double beta);
    AlphaBetaFilter(double alpha, double beta, double initial_position, double initial_velocity);
    double update(double measurement);
};

}