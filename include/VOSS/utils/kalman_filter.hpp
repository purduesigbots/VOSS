#pragma once
#include "angle.hpp"
#include "Eigen/Core"
#include "voss/utils/Pose.hpp"
namespace voss {

class EKalmanFilter {
  public:
    EKalmanFilter(double noise_in_x, double noise_in_y,
                  double noise_in_odom_heading, double noise_in_imu_heading,
                  double init_uncertainty = 0.01) {
        this->state = Eigen::RowVector3<double>::Zero();
        this->P = Eigen::Matrix3d::Identity() * init_uncertainty;

        this->Q(0, 0) = noise_in_x;
        this->Q(1, 1) = noise_in_y;
        this->Q(2, 2) = noise_in_odom_heading;
        this->R(0, 0) = noise_in_imu_heading;
    }

    inline Eigen::Matrix3d get_jacobian_F(double delta_fwd, double delta_str,
                                          double theta) {
        return Eigen::Matrix3d{
            {1, 0, -delta_fwd * sin(theta) - delta_str * cos(theta)},
            {0, 1, delta_fwd * cos(theta) - delta_str * sin(theta)},
            {0, 0, 1}};
    }

    void step(Eigen::Vector3d& x_pred, double delta_fwd, double delta_str,
              double delta_theta, double imu_theta) {
        double theta = x_pred.z();

        auto F = get_jacobian_F(delta_fwd, delta_str, theta);

        x_pred.z() = voss::norm(x_pred.z());

        this->P = F * P * F.transpose() + Q;

        this->state = x_pred;

        double imu_norm = voss::norm(imu_theta);

        Eigen::RowVector3d H = Eigen::RowVector3d{0, 0, 1};

        Eigen::Matrix<double, 1, 1> y(imu_norm - state.z());
        y(0, 0) = shortest_angle(y(0, 0)); // normalize the diff (-pi, pi)

        Eigen::Matrix<double, 1, 1> S = H * P * H.transpose() + R;

        // kalman gain
        Eigen::Matrix<double, 3, 1> K = P * H.transpose() / S(0, 0);

        // update predicted state
        this->state = state + K * y;
        // update covariance
        this->P = (I - K * H) * P;

        this->state.z() = voss::norm(this->state.z()); // (0, 2pi)
    }

    voss::Pose get_predicted_pose() {
        return {state.x(), state.y(), state.z()};
    }

    Eigen::Matrix3d get_covariance() const {
        return P;
    }

    void set_pose(voss::Pose new_pose, double new_uncertainty = 0.01) {
        this->state.x() = new_pose.x;
        this->state.y() = new_pose.y;
        this->state.z() = voss::norm(new_pose.theta.value_or(0.0));
        this->P = Eigen::Matrix3d::Identity() * new_uncertainty;
    }

  private:
    Eigen::Vector3d state; //[x_k, y_k, theta_k]^T

    Eigen::Matrix3d P;                           // covariance matrix
    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero(); // process noise covariance

    Eigen::Matrix<double, 1, 1> R;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity(); // identity matrix

    static inline double shortest_angle(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }
};

} // namespace voss
