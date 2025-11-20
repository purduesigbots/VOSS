#pragma once

#include "SSOV/localizer/OdometryLocalizer.hpp"

#include <cmath>

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

class HoloRobotOdom : public ssov::OdometryLocalizer {
    private:
        pros::MotorGroup front_left_motors;
        pros::MotorGroup front_right_motors;
        pros::MotorGroup back_left_motors;
        pros::MotorGroup back_right_motors;
        pros::Rotation rotation;
        pros::Imu imu;
        double prev_front_left;
        double prev_front_right;
        double prev_back_left;
        double prev_back_right;
        double prev_rot;
        double prev_angle;
        double radial_velocity = 0;
        const double smoothing_factor = 0.3;
        const double motor_tpi = 45.46;
        const double rot_tpi = 4037.04908;
        const double tracking_wheel_offset = 4.716;
        //const double tracking_wheel_offset = 0.0;
    public:
        HoloRobotOdom(std::initializer_list<int8_t> front_left_ports,
                  std::initializer_list<int8_t> front_right_ports,
                  std::initializer_list<int8_t> back_left_ports,
                  std::initializer_list<int8_t> back_right_ports,
                  int8_t rot_port, uint8_t imu_port):
            ssov::OdometryLocalizer({}, 10),
            front_left_motors(front_left_ports),
            front_right_motors(front_right_ports),
            back_left_motors(back_left_ports),
            back_right_motors(back_right_ports),
            rotation(rot_port),
            imu(imu_port) {};
        void calibrate() {
            imu.reset(true);
            rotation.set_data_rate(5);
            prev_front_left = get_front_left_position();
            prev_front_right = get_front_right_position();
            prev_back_left = get_back_left_position();
            prev_back_right = get_back_right_position();
            prev_rot = get_rot_position();
            prev_angle = get_imu_angle();
        }
        double get_front_left_position() const {
            double front_left_position = 0;
            for (const auto &pos : front_left_motors.get_position_all()) {
                front_left_position += pos / (motor_tpi * front_left_motors.size());
            }
            return front_left_position;
        }
        double get_front_left_velocity() const {
            double front_left_velocity = 0;
            for (const auto &vel : front_left_motors.get_actual_velocity_all()) {
                front_left_velocity += vel * 15 / (motor_tpi * front_left_motors.size());
                //left_velocity += vel / left_motors.size();
            }
            return front_left_velocity;
        }
        double get_back_left_position() const {
            double back_left_position = 0;
            for (const auto &pos : back_left_motors.get_position_all()) {
                back_left_position += pos / (motor_tpi * back_left_motors.size());
            }
            return back_left_position;
        }
        double get_back_left_velocity() const {
            double back_left_velocity = 0;
            for (const auto &vel : back_left_motors.get_actual_velocity_all()) {
                back_left_velocity += vel * 15 / (motor_tpi * back_left_motors.size());
                //left_velocity += vel / left_motors.size();
            }
            return back_left_velocity;
        }
        double get_front_right_position() const {
            double front_right_position = 0;
            for (const auto &pos : front_right_motors.get_position_all()) {
                front_right_position += pos / (motor_tpi * front_right_motors.size());
            }
            return front_right_position;
        }
        double get_front_right_velocity() const {
            double front_right_velocity = 0;
            for (const auto &vel : front_right_motors.get_actual_velocity_all()) {
                front_right_velocity += vel * 15 / (motor_tpi * front_right_motors.size());
                //right_velocity += vel / right_motors.size();
            }
            return front_right_velocity;
        }
        double get_back_right_position() const {
            double back_right_position = 0;
            for (const auto &pos : back_right_motors.get_position_all()) {
                back_right_position += pos / (motor_tpi * back_right_motors.size());
            }
            return back_right_position;
        }
        double get_back_right_velocity() const {
            double back_right_velocity = 0;
            for (const auto &vel : back_right_motors.get_actual_velocity_all()) {
                back_right_velocity += vel * 15 / (motor_tpi * back_right_motors.size());
                //right_velocity += vel / right_motors.size();
            }
            return back_right_velocity;
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
            double front_left_pos = get_front_left_position();
            double front_right_pos = get_front_right_position();
            double back_left_pos = get_back_left_position();
            double back_right_pos = get_back_right_position();
            double rot = get_rot_position();
            double angle = get_imu_angle();
            double dx = (front_left_pos - (((front_right_pos-back_left_pos)/-2) - ((back_left_pos-front_left_pos)/2))-(prev_front_left - (((prev_front_right-prev_back_left)/-2) - ((prev_back_left-prev_front_left)/2))));
            double dy = ((front_right_pos-back_left_pos)/-2)-((prev_front_right-prev_back_left)/-2);
            //double dtheta = angle - prev_angle;
            double dtheta = (((front_right_pos + front_right_pos)/2) - ((front_left_pos + back_left_pos)/2)/2 * 9) - prev_angle;
            // double dx = (left_pos - prev_left + right_pos - prev_right) / 2;
            // double dy = (rot - prev_rot) + dtheta * tracking_wheel_offset;
            radial_velocity = dy * 100;
            prev_front_left = front_left_pos;
            prev_front_right = front_right_pos;
            prev_back_left = back_left_pos;
            prev_back_right = back_right_pos;
            prev_rot = rot;
            prev_angle = angle;
            return {dx, dy, dtheta};
        }
        ssov::Pose get_velocities() override {
            double front_left_velocity = get_front_left_velocity();
            double front_right_velocity = get_front_right_velocity();
            double back_left_velocity = get_back_left_velocity();
            double back_right_velocity = get_back_right_velocity();
            double rot_velocity = get_rot_velocity();
            double ang_velocity = get_ang_vel();
            double tangential_velocity = front_left_velocity - ((front_right_velocity-back_left_velocity)/-2) - ((back_left_velocity-front_left_velocity)/2);
            double radial_velocity = (front_right_velocity-back_left_velocity)/-2;
            // double tangential_velocity = (left_velocity + right_velocity) / 2;
            //double radial_velocity = rot_velocity + ang_velocity * tracking_wheel_offset;
            return {tangential_velocity, radial_velocity, ang_velocity};
        }
};