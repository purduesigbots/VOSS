#include "VOSS/trajectory/Profile.hpp"
#include "iostream"
#include "VOSS/utils/Algorithms.hpp"
#include <cmath>
#define DEBUG true

namespace voss::trajectory {

Profile::Profile(std::vector<MotionState> samples): samples(samples) {
    this->times.push_back(0.0);
    for (int i = 1; i < samples.size(); i++) {
        double new_time = this->times.back();
        MotionState current = samples.at(i);
        MotionState prev = samples.at(i - 1);
        new_time += 2 * (current.pos - prev.pos) / (current.vel + prev.vel);
        this->times.push_back(new_time);
    }
}

MotionState Profile::at(double t) {
    if (t < times.front()) {
        MotionState state = samples.front();
        return {state.vel * t, state.vel, 0};
    }
    if (t > times.back()) {
        MotionState state = samples.back();
        return {state.pos + state.vel * (t - times.back()), state.vel, 0};
    }

    int index = voss::utils::insert_index(times, t);

    if (times[index] == t) {
        return samples[index];
    }

    double dt = t - times[index - 1];
    MotionState prev_state = samples[index - 1];
    return {(0.5 * prev_state.acc * dt + prev_state.vel) * dt + prev_state.pos, prev_state.acc * dt + prev_state.vel, prev_state.acc};
}

double Profile::duration() {
    return times.back();
}
}