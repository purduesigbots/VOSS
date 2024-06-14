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
            new_state.vel = constraints.max_vel / (1 + constraints.track_width / 2 * fabs(curvature));
            if (constraints.max_centr_accel) {
                new_state.vel = fmin(new_state.vel, sqrt(constraints.max_centr_accel / fabs(curvature)));
            }
        }
        profile_samples.push_back(new_state);
    }

    profile_samples.front().vel = 0;
    // second pass: acceleration forward pass
    for (int i = 1; i < profile_samples.size(); i++) {
        double prev_vel = profile_samples[i - 1].vel;
        if (prev_vel >= profile_samples[i].vel) {
            profile_samples[i].acc = 0.0;
            continue;
        }

        // initial bounds for velocity. A reasonable minimum is set so velocity never reaches zero.
        double max_vel = profile_samples[i].vel;
        double ds = path_samples.distances[i] - path_samples.distances[i - 1];
        double min_vel = sqrt(2 * constraints.max_accel * ds);

        // apply translational acceleration bounds
        max_vel = fmin(max_vel, sqrt(prev_vel * prev_vel + 2 * constraints.max_accel * ds));
        double min_vel_det = prev_vel * prev_vel + 2 * constraints.max_decel * ds;
        if (min_vel_det > 0.0) {
            min_vel = fmax(min_vel, sqrt(min_vel_det));
        }

        // apply angular acceleration bounds
        // see http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf pages 25-28
        double curvature = path_samples.poses[i].curvature;
        double prev_curvature = path_samples.poses[i - 1].curvature;
        if (curvature != 0.0) {
            double pos_det = (curvature + prev_curvature) * (curvature + prev_curvature) * prev_vel * prev_vel + 8 * curvature * ds * constraints.max_ang_accel;
            double neg_det = (curvature + prev_curvature) * (curvature + prev_curvature) * prev_vel * prev_vel - 8 * curvature * ds * constraints.max_ang_accel;
            if (curvature > 0.0) {
                double vel_1 = ((prev_curvature - curvature) * prev_vel + sqrt(pos_det)) / (2 * curvature);
                if (neg_det < 0.0) {
                    max_vel = fmin(max_vel, vel_1);
                } else {
                    double vel_2 = ((prev_curvature - curvature) * prev_vel + sqrt(neg_det)) / (2 * curvature);
                    if (max_vel >= vel_2) {
                        max_vel = fmin(max_vel, vel_1);
                    } else {
                        vel_2 = ((prev_curvature - curvature) * prev_vel - sqrt(neg_det)) / (2 * curvature);
                        max_vel = fmax(vel_2, min_vel);
                    }
                }
            } else {
                double vel_2 = ((prev_curvature - curvature) * prev_vel - sqrt(neg_det)) / (2 * curvature);
                if (pos_det < 0.0) {
                    max_vel = fmin(max_vel, vel_2);
                } else {
                    double vel_1 = ((prev_curvature - curvature) * prev_vel - sqrt(pos_det)) / (2 * curvature);
                    if (max_vel >= vel_1) {
                        max_vel = fmin(max_vel, vel_2);
                    } else {
                        double vel_1 = ((prev_curvature - curvature) * prev_vel + sqrt(pos_det)) / (2 * curvature);
                        max_vel = fmax(vel_1, min_vel);
                    }
                }
            }
        } else if (prev_curvature != 0.0) {
            double vel_1 = (-2 * constraints.max_ang_accel) / (prev_curvature * prev_vel) - prev_vel;
            double vel_2 = (2 * constraints.max_ang_accel) / (prev_curvature * prev_vel) - prev_vel;
            max_vel = fmin(max_vel, fmax(vel_1, vel_2));
        }

        profile_samples[i].vel = max_vel;
        profile_samples[i - 1].acc = (max_vel * max_vel - prev_vel * prev_vel) / (2 * ds);
    }

    profile_samples.back().vel = 0;
    // third pass: acceleration backward pass
    for (int i = profile_samples.size() - 2; i >= 0; i--) {
        double next_vel = profile_samples[i + 1].vel;
        if (next_vel >= profile_samples[i].vel) continue;

        double max_vel = profile_samples[i].vel;
        double ds = path_samples.distances[i + 1] - path_samples.distances[i];
        double min_vel = sqrt(-2 * constraints.max_decel * ds);

        max_vel = fmin(max_vel, sqrt(next_vel * next_vel - 2 * constraints.max_decel * ds));
        double min_vel_det = next_vel * next_vel - 2 * constraints.max_accel * ds;
        if (min_vel_det > 0.0) {
            min_vel = fmax(min_vel, sqrt(min_vel_det));
        }

        double curvature = path_samples.poses[i].curvature;
        double next_curvature = path_samples.poses[i + 1].curvature;
        if (curvature != 0.0) {
            double pos_det = (curvature + next_curvature) * (curvature + next_curvature) * next_vel * next_vel + 8 * curvature * ds * constraints.max_ang_accel;
            double neg_det = (curvature + next_curvature) * (curvature + next_curvature) * next_vel * next_vel - 8 * curvature * ds * constraints.max_ang_accel;
            if (curvature > 0.0) {
                double vel_1 = ((next_curvature - curvature) * next_vel + sqrt(pos_det)) / (2 * curvature);
                max_vel = fmin(max_vel, fmax(vel_1, min_vel));
            } else {
                double vel_2 = ((next_curvature - curvature) * next_vel - sqrt(neg_det)) / (2 * curvature);
                max_vel = fmin(max_vel, fmax(vel_2, min_vel));
            }
        } else if (next_curvature != 0.0) {
            double vel_1 = (-2 * constraints.max_ang_accel) / (next_curvature * next_vel) - next_vel;
            double vel_2 = (2 * constraints.max_ang_accel) / (next_curvature * next_vel) - next_vel;
            max_vel = fmin(max_vel, fmax(vel_1, vel_2));
        }

        profile_samples[i].vel = max_vel;
        profile_samples[i].acc = (next_vel * next_vel - max_vel * max_vel) / (2 * ds);
    }

    this->profile = Profile(profile_samples);
}

TrajectoryPose Trajectory::at(double t) {
    MotionState state = this->profile.at(t);
    PoseWithCurvature pose = this->path.at(state.pos);
    TrajectoryPose result = {pose.pose, state.vel, state.acc, state.vel * pose.curvature};
    if (path.is_reversed()) {
        result.vel *= -1;
        result.acc *= -1;
    }
    return result;
}

double Trajectory::get_duration() {
    return profile.duration();
}

}