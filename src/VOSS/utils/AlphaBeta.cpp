#include "VOSS/utils/AlphaBeta.hpp"

namespace voss::utils {

AlphaBetaFilter::AlphaBetaFilter(double alpha, double beta): alpha(alpha), beta(beta), position(0.0), velocity(0.0) {
}

AlphaBetaFilter::AlphaBetaFilter(double alpha, double beta, double initial_position, double initial_velocity) : alpha(alpha), beta(beta), position(initial_position), velocity(initial_velocity) {
}

double AlphaBetaFilter::update(double measurement) {
    double predicted_position = position + velocity;
    double residual = measurement - predicted_position;
    position = predicted_position + alpha * residual;
    velocity = velocity + beta * residual;
    return measurement;
}

}