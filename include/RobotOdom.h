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
        const double motor_tpi = 45.46;
        const double rot_tpi = 4037.04908;
        const double tracking_wheel_offset = 4.12293384;
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
        double get_right_position() const {
            double right_position = 0;
            for (const auto &pos : right_motors.get_position_all()) {
                right_position += pos / (motor_tpi * right_motors.size());
            }
            return right_position;
        }
        double get_rot_position() const {
            return rotation.get_position() / rot_tpi;
        }
        double get_imu_angle() const {
            return imu.get_rotation() * -1 * M_PI / 180;
        }
        ssov::Pose get_local_change() override {
            double left_pos = get_left_position();
            double right_pos = get_right_position();
            double rot = get_rot_position();
            double angle = get_imu_angle();
            double dx = (left_pos - prev_left + right_pos - prev_right) / 2;
            double dtheta = angle - prev_angle;
            double dy = (rot - prev_rot) + dtheta * tracking_wheel_offset;
            prev_left = left_pos;
            prev_right = right_pos;
            prev_rot = rot;
            prev_angle = angle;
            return {dx, dy, dtheta};
        }
};