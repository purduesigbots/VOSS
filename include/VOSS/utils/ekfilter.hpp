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
             Eigen::Vector3d init_pose = Eigen::Vector3d::Zero(), double init_identity = 0.0001);

    [[gnu::unused]]void update_trackers(double delta_left, double delta_middle,
                         double delta_heading, double left_dist,
                         double middle_dist);

    void predict(const Eigen::Vector2d& odom_delta, double delta_heading);


    void update_gps_pose_and_error(const Eigen::Vector3d& gps_pose, double gps_error);

    [[nodiscard("Returns current state")]]
    Eigen::Vector3d get_state() const;

    [[nodiscard("Returns odom_init boolean")]]
    bool is_odom_initialized() const;

    [[nodiscard("Returns state covariance matrix")]]
    Eigen::Matrix3d get_covariance() const;

    [[nodiscard("Returns process noise matrix")]]
    Eigen::Matrix3d get_process_noise() const;

    void set_pose(const Eigen::Vector3d& new_pose, double reset_variance = 0.001);

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
