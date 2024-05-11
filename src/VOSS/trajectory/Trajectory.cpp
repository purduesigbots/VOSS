#include "VOSS/trajectory/Trajectory.hpp"

namespace voss::trajectory {

Trajectory::Trajectory(SplinePath path, TrajectoryConstraints constraints): path(path) {
    PathSample path_samples = path.sample(0.25);
    std::vector<MotionState> profile_samples;

    // first pass: fill in distances, max velocities
    for (int i = 0; i < path_samples.distances.size(); i++) {
        MotionState new_state;
        new_state.pos = path_samples.distances[i];
        double curvature = path_samples.poses[i].curvature;
        if (curvature == 0.0) {
            new_state.vel = constraints.max_vel;
        } else {
            new_state.vel = constraints.max_vel / (1 + constraints.track_width / 2 * curvature);
        }
        profile_samples.push_back(new_state);
    }

    profile_samples.front().vel = 0;
    // second pass: acceleration forward pass
    for (int i = 1; i < profile_samples.size(); i++) {
        double vel = profile_samples[i - 1].vel;
        if (vel >= profile_samples[i].vel) {
            profile_samples[i].acc = 0.0;
            continue;
        }
        double max_accel = constraints.max_accel;
        double curvature = path_samples.poses[i].curvature;
        double prev_curvature = path_samples.poses[i - 1].curvature;
        double ds = path_samples.distances[i] - path_samples.distances[i - 1];

        if (curvature != 0.0 && prev_curvature != 0.0) {
            double radius = 1 / curvature;
            double outer_radius = radius + constraints.track_width / 2;
            double dr = (radius - 1 / prev_curvature) / ds * vel;
            max_accel = radius * (max_accel / outer_radius + dr * (vel / (radius * radius) - vel * (1 + constraints.track_width / 2 * curvature) / (outer_radius * outer_radius)));
        }
        double new_vel = vel * vel + 2 * max_accel * ds;
        if (new_vel < 0) {
            new_vel = 0;
            max_accel = -(vel * vel) / (2 * ds);
        }
        new_vel = sqrt(new_vel);

        if (new_vel <= profile_samples[i].vel) {
            profile_samples[i].vel = new_vel;
            profile_samples[i].acc = max_accel;
        } else {
            new_vel = profile_samples[i].vel;
            profile_samples[i].acc = (new_vel * new_vel - vel * vel) / (2 * ds);
        }
    }

    profile_samples.back().vel = 0;
    // third pass: acceleration backward pass
    for (int i = profile_samples.size() - 2; i >= 0; i--) {
        double vel = profile_samples[i + 1].vel;
        if (vel >= profile_samples[i].vel) continue;

        double max_decel = constraints.max_decel;
        double curvature = path_samples.poses[i].curvature;
        double next_curvature = path_samples.poses[i + 1].curvature;
        double ds = path_samples.distances[i + 1] - path_samples.distances[i];

        if (curvature != 0.0 && next_curvature != 0.0) {
            double radius = 1 / curvature;
            double outer_radius = radius + constraints.track_width / 2;
            double dr = (1 / next_curvature - radius) / ds * vel;
            max_decel = radius * (max_decel / outer_radius + dr * (vel / (radius * radius) - vel * (1 + constraints.track_width / 2 * curvature) / (outer_radius * outer_radius)));
        }

        double prev_vel = vel * vel - 2 * max_decel * ds;
        if (prev_vel < 0) {
            prev_vel = 0;
            max_decel = (vel * vel) / (2 * ds);
        }
        prev_vel = sqrt(prev_vel);

        if (prev_vel <= profile_samples[i].vel) {
            profile_samples[i].vel = prev_vel;
            profile_samples[i].acc = max_decel;
        } else {
            prev_vel = profile_samples[i].vel;
            profile_samples[i].acc = (vel * vel - prev_vel * prev_vel) / 2 * ds;
        }
    }

    this->profile = Profile(profile_samples);
}

TrajectoryPose Trajectory::at(double t) {
    MotionState state = this->profile.at(t);
    PoseWithCurvature pose = this->path.at(state.pos);
    return {pose, state.vel, state.acc};
}

}