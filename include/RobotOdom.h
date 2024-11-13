#pragma once

#include "SSOV/localizer/OdometryLocalizer.hpp"

#include <cmath>

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

class RobotOdom : public ssov::OdometryLocalizer {
    private:
        pros::MotorGroup left_motors;
        pros::MotorGroup right_motors;
        pros::Rotation rotation;
        pros::Imu imu;
        double prev_left;
        double prev_right;
        double prev_rot;
        double prev_angle;
        double radial_velocity = 0;
        const double smoothing_factor = 0.3;
        const double motor_tpi = 45.46;
        const double rot_tpi = 4037.04908;
        const double tracking_wheel_offset = 4.61;
    public:
        RobotOdom(std::initializer_list<int8_t> left_ports,
                  std::initializer_list<int8_t> right_ports,
                  int8_t rot_port, uint8_t imu_port):
            ssov::OdometryLocalizer(10),
            left_motors(left_ports),
            right_motors(right_ports),
            rotation(rot_port),
            imu(imu_port) {};
        void calibrate() {
            imu.reset(true);
            rotation.set_data_rate(5);
            prev_left = get_left_position();
            prev_right = get_right_position();
            prev_rot = get_rot_position();
            prev_angle = get_imu_angle();
        }
        double get_left_position() const {
            double left_position = 0;
            for (const auto &pos : left_motors.get_position_all()) {
                left_position += pos / (motor_tpi * left_motors.size());
            }
            return left_position;
        }
        double get_left_velocity() const {
            double left_velocity = 0;
            for (const auto &vel : left_motors.get_actual_velocity_all()) {
                left_velocity += vel * 15 / (motor_tpi * left_motors.size());
                //left_velocity += vel / left_motors.size();
            }
            return left_velocity;
        }
        double get_right_position() const {
            double right_position = 0;
            for (const auto &pos : right_motors.get_position_all()) {
                right_position += pos / (motor_tpi * right_motors.size());
            }
            return right_position;
        }
        double get_right_velocity() const {
            double right_velocity = 0;
            for (const auto &vel : right_motors.get_actual_velocity_all()) {
                right_velocity += vel * 15 / (motor_tpi * right_motors.size());
                //right_velocity += vel / right_motors.size();
            }
            return right_velocity;
        }
        double get_rot_position() const {
            return rotation.get_position() / rot_tpi;
        }
        double get_rot_velocity() const {
            return rotation.get_velocity() * 100 / rot_tpi;
        }
        double get_imu_angle() const {
            return imu.get_rotation() * -1 * M_PI / 180;
        }
        double get_ang_vel() const {
            return imu.get_gyro_rate().z * M_PI / 180;
        }
        ssov::Pose get_local_change() override {
            double left_pos = get_left_position();
            double right_pos = get_right_position();
            double rot = get_rot_position();
            double angle = get_imu_angle();
            double dx = (left_pos - prev_left + right_pos - prev_right) / 2;
            double dtheta = angle - prev_angle;
            double dy = (rot - prev_rot) + dtheta * tracking_wheel_offset;
            radial_velocity = smoothing_factor * (dy * 100) + (1 - smoothing_factor) * radial_velocity;
            prev_left = left_pos;
            prev_right = right_pos;
            prev_rot = rot;
            prev_angle = angle;
            return {dx, dy, dtheta};
        }
        ssov::Pose get_velocities() override {
            double left_velocity = get_left_velocity();
            double right_velocity = get_right_velocity();
            double rot_velocity = get_rot_velocity();
            double ang_velocity = get_ang_vel();
            double tangential_velocity = (left_velocity + right_velocity) / 2;
            //double radial_velocity = rot_velocity + ang_velocity * tracking_wheel_offset;
            return {tangential_velocity, radial_velocity, ang_velocity};
        }
};