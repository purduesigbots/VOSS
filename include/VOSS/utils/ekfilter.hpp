#pragma once
#include "Eigen/Dense"
#include "pros/gps.hpp"
#include "VOSS/utils/angle.hpp"

namespace voss {

class EKFilter {
  public:
    /**
     *
     * The lower the value is the more trust
     * @param odom_x_noise
     * @param odom_y_noise
     * @param imu_theta_noise
     * @param gps_heading_noise_scalar
     * @param init_pose
     */
    EKFilter(double odom_x_noise, double odom_y_noise, double imu_theta_noise,
             double gps_heading_noise_scalar,
             Eigen::Vector3d init_pose = Eigen::Vector3d::Zero(), double init_identity = 0.0001)
        : state(std::move(init_pose)),
          gps_heading_noise_scalar(gps_heading_noise_scalar) {
        P = P.setIdentity() * init_identity;
        Q = Eigen::Matrix3d({{odom_x_noise, 0, 0},
                             {0, odom_y_noise, 0},
                             {0, 0, imu_theta_noise}});
    }

    [[gnu::unused]]void update_trackers(double delta_left, double delta_middle,
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

    void predict(const Eigen::Vector2d& odom_delta, double delta_heading) {
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


    void update_gps_pose_and_error(const Eigen::Vector3d& gps_pose, double gps_error) {
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

    [[nodiscard("Returns current state")]]
    Eigen::Vector3d get_state() const {
        Eigen::Vector3d ret_state = this->state;
        ret_state.z() = voss::norm(state.z());
        return ret_state;
    }

    [[nodiscard("Returns odom_init boolean")]]
    bool is_odom_initialized() const {
        return odom_init;
    }

    [[nodiscard("Returns state covariance matrix")]]
    Eigen::Matrix3d get_covariance() const {
        return P;
    }

    [[nodiscard("Returns process noise matrix")]]
    Eigen::Matrix3d get_process_noise() const {
        return Q;
    }

    void set_pose(const Eigen::Vector3d& new_pose, double reset_variance = 0.001) {
        this->state = new_pose;
        this->state.z() = normalize_angle(this->state.z());
        this->P.setIdentity();
        this->P *= reset_variance;
    }



  private:
    Eigen::Vector3d state; // x, y, theta
    Eigen::Matrix3d P; // State covariance
    Eigen::Matrix3d Q; // Process noise
    double gps_heading_noise_scalar;
    bool odom_init = false;

    //[pi, -pi]
    inline static double normalize_angle(double angle) {
        return atan2(sin(angle), cos(angle));
    }
};

} // namespace voss
