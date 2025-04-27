
#include "ekfilter.hpp"

namespace voss {
EKFilter::EKFilter(double odom_x_noise, double odom_y_noise,
                   double imu_theta_noise, double gps_heading_noise_scalar,
                   Eigen::Vector3d init_pose, double init_identity)
    : state(std::move(init_pose)),
      gps_heading_noise_scalar(gps_heading_noise_scalar) {
    P = P.setIdentity() * init_identity;
    Q = Eigen::Matrix3d(
        {{odom_x_noise, 0, 0}, {0, odom_y_noise, 0}, {0, 0, imu_theta_noise}});
}
void EKFilter::update_trackers(double delta_left, double delta_middle,
                               double delta_heading, double left_dist,
                               double middle_dist) {
    if (!odom_init) {
        this->odom_init = true;
        return;
    }

    constexpr static double eps = 1e-5;
    double delta_str = 0.0;
    double delta_fwd = 0.0;

    // calculate ref frame deltas
    if (std::abs(delta_heading) > eps) {
        double i = sin(delta_heading / 2.0) * 2.0;
        delta_fwd = (delta_left / delta_heading + left_dist) * i;
        delta_str = (delta_middle / delta_heading + middle_dist) * i;
    } else {
        delta_fwd = delta_left;
        delta_str = delta_middle;
    }

    predict({delta_fwd, delta_str}, delta_heading);
}
void EKFilter::predict(const Eigen::Vector2d& odom_delta,
                       double delta_heading) {
    double p = state.z() - delta_heading / 2.0;
    double cos_theta = cos(p);
    double sin_theta = sin(p);

    Eigen::Vector2d global_delta = {
        cos_theta * odom_delta.x() - sin_theta * odom_delta.y(),
        sin_theta * odom_delta.x() + cos_theta * odom_delta.y()};

    state.x() += global_delta.x();
    state.y() += global_delta.y();
    state.z() += delta_heading;
    state.z() = normalize_angle(state.z());

    // odometry kinematics jacobian
    // take derivative of state
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -sin_theta * odom_delta.x() - cos_theta * odom_delta.y();
    F(1, 2) = cos_theta * odom_delta.x() - sin_theta * odom_delta.y();

    this->P = F * P * F.transpose() + Q;
}
void EKFilter::update_gps_pose_and_error(const Eigen::Vector3d& gps_pose,
                                         double gps_error) {
    /*
     *  y_t = z_t - h_(x_{t|t-1})
     *
     *  S_t = (H_t * P_{t|t-1} * H_t^t + R_t).inverse
     *  K_t = P_{t|t-1} * H_t^T * S
     *
     *  P_t = (I - K_t * H_t) * P_{t|t-1}
     */
    if (gps_error < 0) {
        return;
    }
    double gps_variance = gps_error * gps_error;

    Eigen::Vector3d z = gps_pose;
    z.z() = normalize_angle(z.z());
    Eigen::Vector3d h = state;

    // measurement jacobian
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R = Eigen::Matrix3d{
        {gps_variance, 0, 0},
        {0, gps_variance, 0},
        {0, 0, gps_variance * gps_heading_noise_scalar},
    };

    // innovation matrix, the amount of errors
    Eigen::Vector3d y = z - h;

    y.z() = normalize_angle(y.z());

    // kalman gain
    Eigen::Matrix3d S = H * P * H.transpose() + R;
    Eigen::Matrix3d K = P * H.transpose() * S.inverse();

    Eigen::Vector3d new_state = state + K * y;
    new_state.z() = normalize_angle(new_state.z());
    P = (Eigen::Matrix3d::Identity() - K * H) * P;

    this->state = new_state;
}

Eigen::Vector3d EKFilter::get_state() const {
    Eigen::Vector3d ret_state = this->state;
    ret_state.z() = voss::norm(state.z());
    return ret_state;
}

bool EKFilter::is_odom_initialized() const {
    return odom_init;
}

Eigen::Matrix3d EKFilter::get_covariance() const {
    return P;
}

Eigen::Matrix3d EKFilter::get_process_noise() const {
    return Q;
}

void EKFilter::set_pose(const Eigen::Vector3d& new_pose,
                        double reset_variance) {
    this->state = new_pose;
    this->state.z() = normalize_angle(this->state.z());
    this->P.setIdentity();
    this->P *= reset_variance;
}

} // namespace voss